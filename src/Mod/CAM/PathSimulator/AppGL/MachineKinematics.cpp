// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2026 The FreeCAD project association AISBL              *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

#include "MachineKinematics.h"

#include <algorithm>
#include <cmath>

namespace CAMSimulator
{

namespace
{

constexpr double Pi = 3.14159265358979323846;
constexpr double Deg = Pi / 180;

double Dot(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void Cross(const double a[3], const double b[3], double out[3])
{
    const double c[3] = {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    };
    std::copy(c, c + 3, out);
}

// v turned by angle (radians) about the unit vector u, right handed (Rodrigues)
void Rotate(const double u[3], double angle, const double v[3], double out[3])
{
    const double c = std::cos(angle), s = std::sin(angle);
    double uxv[3];
    Cross(u, v, uxv);
    const double uv = Dot(u, v);
    double r[3];
    for (int k = 0; k < 3; k++) {
        r[k] = v[k] * c + uxv[k] * s + u[k] * uv * (1 - c);
    }
    std::copy(r, r + 3, out);
}

// The angle (radians) that turns p about the unit vector u to q, of the same height along u and
// distance from it (Paden-Kahan subproblem 1); 0 for p on u
double TurnAngle(const double u[3], const double p[3], const double q[3])
{
    double pp[3], qq[3];
    const double up = Dot(u, p), uq = Dot(u, q);
    for (int k = 0; k < 3; k++) {
        pp[k] = p[k] - u[k] * up;
        qq[k] = q[k] - u[k] * uq;
    }
    if (Dot(pp, pp) < 1e-18 || Dot(qq, qq) < 1e-18) {
        return 0;
    }
    double c[3];
    Cross(pp, qq, c);
    return std::atan2(Dot(u, c), Dot(pp, qq));
}

// The angles (radians) that make exp(w1 a1) exp(w2 a2) p = q, two unit vectors through the origin
// that aren't parallel (Paden-Kahan subproblem 2): none, one or two solutions
std::vector<std::pair<double, double>> TwoTurns(
    const double w1[3],
    const double w2[3],
    const double p[3],
    const double q[3]
)
{
    std::vector<std::pair<double, double>> out;
    const double w12 = Dot(w1, w2);
    const double denom = w12 * w12 - 1;
    if (std::abs(denom) < 1e-12) {
        return out;
    }
    const double alpha = (w12 * Dot(w2, p) - Dot(w1, q)) / denom;
    const double beta = (w12 * Dot(w1, q) - Dot(w2, p)) / denom;
    double c[3];
    Cross(w1, w2, c);
    const double gamma2 = (Dot(p, p) - alpha * alpha - beta * beta - 2 * alpha * beta * w12) / Dot(c, c);
    if (gamma2 < -1e-9) {
        return out;
    }
    const double gamma = std::sqrt(std::max(0.0, gamma2));
    for (double sign : {1.0, -1.0}) {
        double z[3];
        for (int k = 0; k < 3; k++) {
            z[k] = alpha * w1[k] + beta * w2[k] + sign * gamma * c[k];
        }
        out.emplace_back(TurnAngle(w1, z, q), TurnAngle(w2, p, z));
        if (gamma < 1e-9) {
            break;
        }
    }
    return out;
}

// The angle equal to a (degrees, modulo 360) within the limits of an axis nearest to 'near', false
// if none is within them (then the one nearest to near)
bool WithinLimits(const MachineAxis& axis, double a, double near, double& out)
{
    double best = 0;
    bool found = false;
    for (int k = -3; k <= 3; k++) {
        const double v = a + 360.0 * k;
        if (v < axis.min - 1e-6 || v > axis.max + 1e-6) {
            continue;
        }
        if (!found || std::abs(v - near) < std::abs(best - near)) {
            best = v;
            found = true;
        }
    }
    if (!found) {
        best = a + 360.0 * std::round((near - a) / 360.0);
    }
    out = best;
    return found;
}

}  // namespace

void MachineKinematics::SetAxes(const std::vector<MachineAxis>& linear, const std::vector<MachineAxis>& rotary)
{
    static const char* names[3] = {"X", "Y", "Z"};
    for (int k = 0; k < 3; k++) {
        mHasLinear[k] = false;
        for (const MachineAxis& a : linear) {
            if (a.name == names[k]) {
                mLinear[k] = a;
                mHasLinear[k] = true;
            }
        }
    }
    mRotary = rotary;
    for (MachineAxis& a : mRotary) {
        const double n = std::sqrt(Dot(a.vector, a.vector));
        for (double& v : a.vector) {
            v = n > 0 ? v / n : 0;
        }
    }
    std::stable_sort(mRotary.begin(), mRotary.end(), [](const MachineAxis& a, const MachineAxis& b) {
        return a.table != b.table ? a.table : a.sequence < b.sequence;
    });
    mKnown = true;
}

void MachineKinematics::ToolAxis(const std::vector<double>& angles, double axis[3]) const
{
    double v[3] = {0, 0, 1};
    // the head nearest to the tool first
    for (size_t i = 0; i < mRotary.size(); i++) {
        if (!mRotary[i].table) {
            Rotate(mRotary[i].vector, angles[i] * Deg, v, v);
        }
    }
    // in the part: the table farthest from it undone first
    for (size_t i = mRotary.size(); i-- > 0;) {
        if (mRotary[i].table) {
            Rotate(mRotary[i].vector, -angles[i] * Deg, v, v);
        }
    }
    std::copy(v, v + 3, axis);
}

void MachineKinematics::MachinePoint(const double p[3], const std::vector<double>& angles, double out[3]) const
{
    double v[3] = {p[0], p[1], p[2]};
    for (size_t i = 0; i < mRotary.size(); i++) {
        if (mRotary[i].table) {
            Rotate(mRotary[i].vector, angles[i] * Deg, v, v);
        }
    }
    std::copy(v, v + 3, out);
}

void MachineKinematics::PartPoint(const double m[3], const std::vector<double>& angles, double out[3]) const
{
    double v[3] = {m[0], m[1], m[2]};
    for (size_t i = mRotary.size(); i-- > 0;) {
        if (mRotary[i].table) {
            Rotate(mRotary[i].vector, -angles[i] * Deg, v, v);
        }
    }
    std::copy(v, v + 3, out);
}

bool MachineKinematics::AnglesFor(
    const double axis[3],
    const std::vector<double>& near,
    std::vector<double>& angles,
    bool& withinLimits
) const
{
    const size_t n = mRotary.size();
    const double z[3] = {0, 0, 1};
    angles.assign(n, 0.0);
    withinLimits = true;
    // the solutions in radians, in the order of mRotary
    std::vector<std::vector<double>> solutions;
    if (n == 0) {
        return Dot(axis, z) > 1 - 1e-9;
    }
    if (n == 1) {
        const MachineAxis& a = mRotary[0];
        // the tool turned along: z by the head, or the direction by the table back to z
        const double* from = a.table ? axis : z;
        const double* to = a.table ? z : axis;
        if (std::abs(Dot(a.vector, from) - Dot(a.vector, to)) > 1e-6) {
            return false;
        }
        solutions.push_back({TurnAngle(a.vector, from, to)});
    }
    else {
        // two axes: exp(w1 a1) exp(w2 a2) p = q
        const MachineAxis& a0 = mRotary[0];
        const MachineAxis& a1 = mRotary[1];
        if (a0.table && a1.table) {
            // R1 R0 t = z
            for (const auto& [t1, t0] : TwoTurns(a1.vector, a0.vector, axis, z)) {
                solutions.push_back({t0, t1});
            }
        }
        else if (!a0.table && !a1.table) {
            // H1 H0 z = t
            for (const auto& [t1, t0] : TwoTurns(a1.vector, a0.vector, z, axis)) {
                solutions.push_back({t0, t1});
            }
        }
        else {
            // a table (first) and a head: H^-1 R t = z
            for (const auto& [th, tt] : TwoTurns(a1.vector, a0.vector, axis, z)) {
                solutions.push_back({tt, -th});
            }
        }
        // further axes stay at 0
        for (auto& s : solutions) {
            s.resize(n, 0.0);
        }
    }
    if (solutions.empty()) {
        return false;
    }
    // the solution within the limits nearest to 'near', else the nearest
    double bestCost = 1e300;
    bool bestWithin = false;
    for (const auto& s : solutions) {
        std::vector<double> deg(n);
        bool within = true;
        double cost = 0;
        for (size_t i = 0; i < n; i++) {
            const double from = i < near.size() ? near[i] : 0.0;
            within = WithinLimits(mRotary[i], s[i] / Deg, from, deg[i]) && within;
            cost += std::abs(deg[i] - from);
        }
        if ((within && !bestWithin) || (within == bestWithin && cost < bestCost)) {
            angles = deg;
            bestCost = cost;
            bestWithin = within;
        }
    }
    withinLimits = bestWithin;
    return true;
}

std::vector<Material::Event> MachineEvents(
    const MachineKinematics& machine,
    const MillMotion& start,
    const std::vector<MillMotion>& ends
)
{
    std::vector<Material::Event> events;
    if (!machine.Known()) {
        return events;
    }
    static const char* names[3] = {"X", "Y", "Z"};
    double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
    bool reported[3] = {false, false, false};
    std::vector<double> angles;
    double previous[3];
    MotionToolAxis(start, previous);
    for (size_t i = 0; i <= ends.size(); i++) {
        const MillMotion& m = i == 0 ? start : ends[i - 1];
        const int segment = i == 0 ? 0 : (int)i - 1;
        if (ends.empty()) {
            break;
        }
        double axis[3];
        MotionToolAxis(m, axis);
        // moves turning the tool, like the swings between work planes of the simulator, are left
        // out: the machine turns its rotary axes its own way, the moves after them check where
        // they go
        const bool turning = axis[0] * previous[0] + axis[1] * previous[1] + axis[2] * previous[2]
            < 1 - 1e-9;
        std::copy(axis, axis + 3, previous);
        if (turning) {
            continue;
        }
        std::vector<double> found;
        bool within = false;
        const bool reached = machine.AnglesFor(axis, angles, found, within);
        if (reached) {
            angles = found;
        }
        const double tip[3] = {m.x, m.y, m.z};
        if (!reached || !within) {
            // a direction the rotary axes don't reach, with the moves before it of the same kind
            Material::Event* last = events.empty() ? nullptr : &events.back();
            if (last && last->axis.empty() && last->lastSegment >= segment - 1
                && last->operation == ends[segment].operation) {
                last->lastSegment = segment;
            }
            else {
                Material::Event e;
                e.kind = Material::Event::MachineLimit;
                e.firstSegment = e.lastSegment = segment;
                e.operation = ends[segment].operation;
                std::copy(tip, tip + 3, e.position);
                std::copy(axis, axis + 3, e.direction);
                events.push_back(e);
            }
        }
        // the travel of the linear axes: the tip in the machine, about the origin of the part
        double p[3];
        machine.MachinePoint(tip, angles.empty() ? std::vector<double>(machine.Rotaries().size(), 0.0) : angles, p);
        for (int k = 0; k < 3; k++) {
            lo[k] = std::min(lo[k], p[k]);
            hi[k] = std::max(hi[k], p[k]);
            const MachineAxis* a = machine.Linear(k);
            if (!a || reported[k] || a->max - a->min > 1e20) {
                continue;
            }
            if (hi[k] - lo[k] > a->max - a->min + 1e-6) {
                reported[k] = true;
                Material::Event e;
                e.kind = Material::Event::MachineLimit;
                e.firstSegment = e.lastSegment = segment;
                e.operation = ends[segment].operation;
                std::copy(tip, tip + 3, e.position);
                e.axis = names[k];
                e.needed = hi[k] - lo[k];
                e.available = a->max - a->min;
                events.push_back(e);
            }
        }
    }
    return events;
}

}  // namespace CAMSimulator
