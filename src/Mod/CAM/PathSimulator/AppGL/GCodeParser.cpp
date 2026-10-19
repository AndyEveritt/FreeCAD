// SPDX-License-Identifier: LGPL-2.1-or-later
// SPDX-FileCopyrightText: 2024 Shai Seger <shaise at gmail>
// SPDX-FileNotice: Part of the FreeCAD project.

/******************************************************************************
 *                                                                            *
 *   FreeCAD is free software: you can redistribute it and/or modify          *
 *   it under the terms of the GNU Lesser General Public License as           *
 *   published by the Free Software Foundation, either version 2.1            *
 *   of the License, or (at your option) any later version.                   *
 *                                                                            *
 *   FreeCAD is distributed in the hope that it will be useful,               *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty              *
 *   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *   See the GNU Lesser General Public License for more details.              *
 *                                                                            *
 *   You should have received a copy of the GNU Lesser General Public         *
 *   License along with FreeCAD. If not, see https://www.gnu.org/licenses     *
 *                                                                            *
 ******************************************************************************/

#ifdef _MSC_VER
# ifndef _CRT_SECURE_NO_WARNINGS
#  define _CRT_SECURE_NO_WARNINGS
# endif
#endif

#include "GCodeParser.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace CAMSimulator
{

namespace
{

constexpr double Pi = 3.14159265358979323846;

// A number at p: an optional sign, digits and a decimal point, independent of the locale. Returns
// the end of the number, p if there is none.
const char* ParseNumber(const char* p, double& value)
{
    const char* s = p;
    while (*s == ' ' || *s == '\t') {
        s++;
    }
    double sign = 1;
    if (*s == '+' || *s == '-') {
        sign = *s == '-' ? -1 : 1;
        s++;
    }
    uint64_t mantissa = 0;
    int decimals = 0;
    int digits = 0;
    bool point = false;
    for (;; s++) {
        if (*s >= '0' && *s <= '9') {
            // digits beyond the precision of a double only scale the number
            if (mantissa < 100000000000000000ULL) {
                mantissa = mantissa * 10 + (uint64_t)(*s - '0');
                decimals += point ? 1 : 0;
            }
            else if (!point) {
                decimals--;
            }
            digits++;
        }
        else if (*s == '.' && !point) {
            point = true;
        }
        else {
            break;
        }
    }
    if (digits == 0) {
        return p;
    }
    value = sign * (double)mantissa / std::pow(10.0, decimals);
    return s;
}

}  // namespace

GCodeParser::~GCodeParser() = default;

void GCodeParser::Clear()
{
    Operations.clear();
    lastState = {};
    currentOperation = -1;
    mMode = ModeNone;
    mPlane = 17;
    mIncremental = false;
    mUnit = 1;
    mReturnToR = false;
    mCycleInitial = mCycleR = mCycleDepth = 0;
}

bool GCodeParser::Parse(const char* filename)
{
    Clear();

    FILE* fl = std::fopen(filename, "rt");
    if (fl == nullptr) {
        return false;
    }
    std::string line;
    int source = 0;
    for (int ch; (ch = std::fgetc(fl)) != EOF;) {
        if (ch == '\n') {
            AddLine(line.c_str(), source++);
            line.clear();
        }
        else {
            line.push_back((char)ch);
        }
    }
    AddLine(line.c_str(), source);
    std::fclose(fl);
    return true;
}

void GCodeParser::Tokenize(const char* text, std::vector<Word>& words)
{
    const char* p = text;
    while (*p != 0) {
        const char ch = *p;
        if (ch == '(') {
            // a comment, to its end
            while (*p != 0 && *p != ')') {
                p++;
            }
            if (*p != 0) {
                p++;
            }
            continue;
        }
        if (ch == ';') {
            break;  // a comment to the end of the line
        }
        p++;
        if (std::isalpha((unsigned char)ch) == 0) {
            continue;
        }
        double value = 0;
        const char* end = ParseNumber(p, value);
        if (end != p) {
            words.push_back({(char)std::toupper((unsigned char)ch), value});
            p = end;
        }
    }
}

bool GCodeParser::AddLine(const char* line, int source)
{
    std::vector<Word> words;
    Tokenize(line, words);
    // feed rates of G-code text are per minute
    return Apply(words, 1.0 / 60, source);
}

bool GCodeParser::AddCommand(
    const std::string& name,
    const std::map<std::string, double>& params,
    int source
)
{
    std::vector<Word> words;
    Tokenize(name.c_str(), words);
    for (const auto& [key, value] : params) {
        if (key.size() == 1 && std::isalpha((unsigned char)key[0]) != 0) {
            words.push_back({(char)std::toupper((unsigned char)key[0]), value});
        }
    }
    return Apply(words, 1.0, source);
}

bool GCodeParser::Apply(const std::vector<Word>& words, double feedScale, int source)
{
    // positions given: X, Y, Z, A, B, C
    static constexpr char Axes[] = "XYZABC";
    bool has[6] = {};
    double value[6] = {};
    bool hasCenter[3] = {};
    double center[3] = {};
    bool hasR = false;
    double r = 0;
    bool cycleStart = false;
    // the axis words of these commands are not positions to move to: home (G28, G30), machine
    // coordinates (G53), offsets (G10, G92)
    bool notAMove = false;

    for (const Word& w : words) {
        switch (w.letter) {
            case 'G': {
                const long g = std::lround(w.value * 10);
                switch (g) {
                    case 0:
                        mMode = ModeRapid;
                        break;
                    case 10:
                        mMode = ModeLine;
                        break;
                    case 20:
                        mMode = ModeArcCW;
                        break;
                    case 30:
                        mMode = ModeArcCCW;
                        break;
                    case 170:
                    case 180:
                    case 190:
                        mPlane = (int)(g / 10);
                        break;
                    case 200:
                        mUnit = 25.4;
                        break;
                    case 210:
                        mUnit = 1;
                        break;
                    case 900:
                        mIncremental = false;
                        break;
                    case 910:
                        mIncremental = true;
                        break;
                    case 800:
                        mMode = ModeNone;
                        break;
                    case 980:
                        mReturnToR = false;
                        break;
                    case 990:
                        mReturnToR = true;
                        break;
                    case 100:
                    case 280:
                    case 300:
                    case 530:
                    case 920:
                        notAMove = true;
                        break;
                    case 730:
                    case 740:
                    case 760:
                    case 810:
                    case 820:
                    case 830:
                    case 840:
                    case 850:
                    case 860:
                    case 870:
                    case 880:
                    case 890:
                        cycleStart = mMode != ModeCycle;
                        mMode = ModeCycle;
                        break;
                    default:
                        break;
                }
                break;
            }
            case 'F':
                lastState.feed = w.value * mUnit * feedScale;
                break;
            case 'T':
                lastState.tool = (int)std::lround(w.value);
                break;
            case 'I':
            case 'J':
            case 'K':
                hasCenter[w.letter - 'I'] = true;
                center[w.letter - 'I'] = w.value * mUnit;
                break;
            case 'R':
                hasR = true;
                r = w.value * mUnit;
                break;
            default:
                for (int k = 0; k < 6; k++) {
                    if (w.letter == Axes[k]) {
                        has[k] = true;
                        value[k] = w.value;
                    }
                }
                break;
        }
    }
    if (notAMove || !(has[0] || has[1] || has[2] || has[3] || has[4] || has[5])) {
        return false;
    }

    MillMotion target = lastState;
    target.cmd = eMoveLiner;
    target.i = target.j = target.k = 0;
    target.rapid = mMode == ModeRapid;
    target.operation = currentOperation;
    target.source = source;
    double* coords[6] = {&target.x, &target.y, &target.z, &target.a, &target.b, &target.c};
    for (int k = 0; k < 6; k++) {
        if (has[k]) {
            // positions in the units, angles in degrees
            const double v = k < 3 ? value[k] * mUnit : value[k];
            *coords[k] = mIncremental ? *coords[k] + v : v;
        }
    }

    auto push = [this](const MillMotion& m) {
        lastState = m;
        Operations.push_back(m);
    };

    if (mMode == ModeCycle) {
        // the hole at target x, y: Z (the depth) and R are modal
        if (cycleStart) {
            mCycleInitial = lastState.z;
            mCycleR = lastState.z;
            mCycleDepth = lastState.z;
        }
        if (has[2]) {
            mCycleDepth = target.z;
        }
        if (hasR) {
            mCycleR = r;
        }
        const double above = std::max(lastState.z, mCycleR);
        const double back = mReturnToR ? mCycleR : std::max(mCycleInitial, mCycleR);
        MillMotion m = lastState;
        m.cmd = eMoveLiner;
        m.i = m.j = m.k = 0;
        m.operation = currentOperation;
        m.source = source;
        m.rapid = true;
        if (lastState.z < above) {
            m.z = above;
            push(m);
        }
        m.x = target.x;
        m.y = target.y;
        m.a = target.a;
        m.b = target.b;
        m.c = target.c;
        push(m);
        m.z = mCycleR;
        push(m);
        m.rapid = false;
        m.z = mCycleDepth;
        push(m);
        m.rapid = true;
        m.z = back;
        push(m);
        return true;
    }

    if (mMode == ModeArcCW || mMode == ModeArcCCW) {
        const bool clockwise = mMode == ModeArcCW;
        // the plane of the arc: its axes u, v and the axis w along its normal
        const int u = mPlane == 18 ? 2 : mPlane == 19 ? 1 : 0;
        const int v = mPlane == 18 ? 0 : mPlane == 19 ? 2 : 1;
        const double start[3] = {lastState.x, lastState.y, lastState.z};
        const double end[3] = {target.x, target.y, target.z};
        double cu = 0, cv = 0;
        if (hasCenter[u] || hasCenter[v] || !hasR) {
            cu = start[u] + center[u];
            cv = start[v] + center[v];
        }
        else {
            // by the radius: the center on the side of the chord of the direction, beyond it for
            // a negative radius (more than a half circle)
            const double du = end[u] - start[u], dv = end[v] - start[v];
            const double d = std::hypot(du, dv);
            if (d < 1e-12) {
                return false;
            }
            const double h = std::sqrt(std::max(0.0, r * r - d * d / 4));
            const double side = (clockwise ? -1 : 1) * (r > 0 ? 1 : -1);
            cu = (start[u] + end[u]) / 2 - side * h * dv / d;
            cv = (start[v] + end[v]) / 2 + side * h * du / d;
        }
        if (std::hypot(start[u] - cu, start[v] - cv) < 1e-9) {
            push(target);  // no arc around the start: a line
            return true;
        }
        if (mPlane == 17) {
            target.cmd = clockwise ? eRotateCW : eRotateCCW;
            target.i = cu - start[0];
            target.j = cv - start[1];
            push(target);
            return true;
        }
        double c[3] = {0, 0, 0};
        c[u] = cu;
        c[v] = cv;
        AddArc(target, clockwise, c[0] - start[0], c[1] - start[1], c[2] - start[2]);
        return true;
    }

    push(target);
    return true;
}

// An arc outside the XY plane, in lines from the last position to end within ArcTolerance
void GCodeParser::AddArc(const MillMotion& end, bool clockwise, double ci, double cj, double ck)
{
    const int u = mPlane == 18 ? 2 : 1;
    const int v = mPlane == 18 ? 0 : 2;
    const int w = mPlane == 18 ? 1 : 0;
    const MillMotion start = lastState;
    const double p0[3] = {start.x, start.y, start.z};
    const double p1[3] = {end.x, end.y, end.z};
    const double c[3] = {start.x + ci, start.y + cj, start.z + ck};
    const double radius = std::hypot(p0[u] - c[u], p0[v] - c[v]);
    const double a0 = std::atan2(p0[v] - c[v], p0[u] - c[u]);
    const double a1 = std::atan2(p1[v] - c[v], p1[u] - c[u]);
    double sweep = a1 - a0;
    if (clockwise) {
        while (sweep >= -1e-12) {
            sweep -= 2 * Pi;
        }
    }
    else {
        while (sweep <= 1e-12) {
            sweep += 2 * Pi;
        }
    }
    const double maxStep = radius > ArcTolerance ? 2 * std::acos(1 - ArcTolerance / radius) : Pi / 2;
    const int n = std::max(1, (int)std::ceil(std::abs(sweep) / maxStep));
    for (int i = 1; i <= n; i++) {
        MillMotion m = end;
        if (i < n) {
            const double t = (double)i / n;
            const double a = a0 + t * sweep;
            double p[3];
            p[u] = c[u] + radius * std::cos(a);
            p[v] = c[v] + radius * std::sin(a);
            p[w] = p0[w] + t * (p1[w] - p0[w]);
            m.x = p[0];
            m.y = p[1];
            m.z = p[2];
            m.a = start.a + t * (end.a - start.a);
            m.b = start.b + t * (end.b - start.b);
            m.c = start.c + t * (end.c - start.c);
        }
        m.cmd = eMoveLiner;
        m.i = m.j = m.k = 0;
        lastState = m;
        Operations.push_back(m);
    }
}

}  // namespace CAMSimulator
