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


#include "DexelStock.h"
#include "ThreadPool.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace CAMSimulator
{

namespace
{

constexpr float MinThickness = 1e-4f;

// a - r for sorted, disjoint span lists. Returns the number of spans written to out, which must
// have room for na + nr spans. Ends created by r get the normals of r.
int Subtract(const Span* a, int na, const Span* r, int nr, Span* out)
{
    int n = 0;
    int j = 0;
    for (int i = 0; i < na; i++) {
        float lo = a[i].lo;
        uint32_t nlo = a[i].nlo;
        const float hi = a[i].hi;
        while (j < nr && r[j].hi <= lo) {
            j++;
        }
        for (int k = j; k < nr && r[k].lo < hi; k++) {
            if (r[k].lo > lo + MinThickness) {
                out[n++] = {lo, r[k].lo, nlo, r[k].nlo};
            }
            if (r[k].hi > lo) {
                lo = r[k].hi;
                nlo = r[k].nhi;
            }
            if (lo >= hi) {
                break;
            }
        }
        if (hi - lo > MinThickness) {
            out[n++] = {lo, hi, nlo, a[i].nhi};
        }
    }
    return n;
}

bool IsVertical(const float a[3])
{
    return std::abs(a[0]) < 1e-6f && std::abs(a[1]) < 1e-6f && a[2] > 0;
}

// Packed material normal for the outward normal of the tool surface, n = (r * h, z) with the
// unit vector h pointing horizontally away from the tool axis
uint32_t ToolNormal(const ProfileNormal& pn, float hx, float hy)
{
    const float n[3] = {-pn.r * hx, -pn.r * hy, -pn.z};
    return PackNormal(n, true);
}

uint32_t ToolNormal3(const float* n)
{
    const float m[3] = {-n[0], -n[1], -n[2]};
    return PackNormal(m, true);
}

// Chord of a line with the convex hull of two disks: the union of the disks with center and radius
// interpolated linearly between (c0, r0) and (c1, r1), the slices of a tool moving linearly while
// its radius at the height of the line changes linearly. In a 2D frame, s along the line, w
// across. Returns the range of s, and the interpolation parameters of its ends.
bool HullChord(
    const float c0[2],
    float r0,
    const float c1[2],
    float r1,
    float w,
    float& lo,
    float& hi,
    float& lambdaLo,
    float& lambdaHi
)
{
    // offset of the line from the center d = d0 + l * dd, radius r = r0 + l * dr, the half
    // chord is sqrt(Q) with Q = (r - d) (r + d) = A + B l + C l^2
    const double d0 = w - c0[1], dd = -(double)(c1[1] - c0[1]);
    const double dr = (double)r1 - r0;
    // where the line touches the disks: r - d >= 0 and r + d >= 0
    double l0 = 0, l1 = 1;
    auto limit = [&](double a, double b) {  // a + b * l >= 0
        if (std::abs(b) < 1e-15) {
            if (a < 0) {
                l0 = 1;
                l1 = 0;
            }
            return;
        }
        const double x = -a / b;
        if (b > 0) {
            l0 = std::max(l0, x);
        }
        else {
            l1 = std::min(l1, x);
        }
    };
    limit(r0 - d0, dr - dd);
    limit(r0 + d0, dr + dd);
    if (l0 > l1) {
        return false;
    }
    const double s0 = c0[0], ds = (double)c1[0] - c0[0];
    const double A = ((double)r0 - d0) * ((double)r0 + d0);
    const double B = (dr - dd) * (r0 + d0) + (dr + dd) * (r0 - d0);
    const double C = (dr - dd) * (dr + dd);
    // stationary points of s(l) +- sqrt(Q(l)): (B + 2 C l)^2 = 4 ds^2 Q(l), a double root where
    // the line passes the path of the center, which rounding can turn into none: the vertex of
    // the quadratic is a candidate then. The widest chord is a candidate as well.
    double candidates[5] = {l0, l1, -1, -1, -1};
    const double qa = C * (C - ds * ds), qb = B * (C - ds * ds), qc = B * B / 4 - ds * ds * A;
    if (std::abs(qa) > 1e-15) {
        const double disc = qb * qb - 4 * qa * qc;
        if (disc >= 0) {
            const double sq = std::sqrt(disc);
            candidates[2] = (-qb - sq) / (2 * qa);
            candidates[3] = (-qb + sq) / (2 * qa);
        }
        else {
            candidates[2] = -qb / (2 * qa);
        }
    }
    else if (std::abs(qb) > 1e-15) {
        candidates[2] = -qc / qb;
    }
    if (std::abs(C) > 1e-15) {
        candidates[4] = std::clamp(-B / (2 * C), l0, l1);
    }
    double best[2] = {1e300, -1e300};
    for (double l : candidates) {
        if (l < l0 || l > l1) {
            continue;
        }
        const double half = std::sqrt(std::max(0.0, A + B * l + C * l * l));
        const double s = s0 + ds * l;
        if (s - half < best[0]) {
            best[0] = s - half;
            lambdaLo = (float)l;
        }
        if (s + half > best[1]) {
            best[1] = s + half;
            lambdaHi = (float)l;
        }
    }
    lo = (float)best[0];
    hi = (float)best[1];
    return true;
}

// True if there is material of the sorted, disjoint spans in [lo, hi]
bool MaterialIn(const Span* spans, int n, float lo, float hi)
{
    const Span* it = std::upper_bound(spans, spans + n, lo, [](float x, const Span& s) {
        return x < s.hi;
    });
    return it != spans + n && it->lo < hi;
}

// True if x is inside the material of the sorted, disjoint spans, not at one of its ends: an end of
// a removal there becomes an end of the material
bool InsideMaterial(const Span* spans, int n, float x)
{
    const Span* it = std::upper_bound(spans, spans + n, x, [](float v, const Span& s) {
        return v < s.hi;
    });
    return it != spans + n && it->lo < x;
}

// A half plane a * s + b * t + c >= 0, with (a, b) a unit vector
struct HalfPlane
{
    double a, b, c;
};

// The range of s over the points (s, t) where the quadratic
// q[0] s^2 + 2 q[1] s t + q[2] t^2 + 2 q[3] s + 2 q[4] t + q[5] is at most 0, inside the half planes,
// and the t of its ends. The set must be convex and bounded in s. Its extremes in s are corners of
// the half planes, points where their lines cross the conic, or points of the conic tangent to s.
// Where an extreme is reached along a range of t, like on a flat face of the tool, the t of the
// middle of it, away from the edges of the face. False if the set is empty. tol is the tolerance
// of the tests, a distance.
bool ConvexRange(
    const double q[6],
    const HalfPlane* planes,
    int n,
    double tol,
    double& sMin,
    double& tMin,
    double& sMax,
    double& tMax
)
{
    auto quadric = [&](double s, double t) {
        return q[0] * s * s + 2 * q[1] * s * t + q[2] * t * t + 2 * q[3] * s + 2 * q[4] * t + q[5];
    };
    bool found = false;
    // the range of t at each extreme
    double minT0 = 0, minT1 = 0, maxT0 = 0, maxT1 = 0;
    const double same = 10 * tol;
    auto consider = [&](double s, double t) {
        if (found && s > sMin + same && s < sMax - same) {
            return;  // can't change the range
        }
        for (int i = 0; i < n; i++) {
            if (planes[i].a * s + planes[i].b * t + planes[i].c < -tol) {
                return;
            }
        }
        const double gs = q[0] * s + q[1] * t + q[3], gt = q[1] * s + q[2] * t + q[4];
        if (quadric(s, t) > 2 * tol * std::sqrt(gs * gs + gt * gt) + tol * tol) {
            return;
        }
        if (!found || s < sMin - same) {
            sMin = s;
            minT0 = minT1 = t;
        }
        else if (s <= sMin + same) {
            sMin = std::min(sMin, s);
            minT0 = std::min(minT0, t);
            minT1 = std::max(minT1, t);
        }
        if (!found || s > sMax + same) {
            sMax = s;
            maxT0 = maxT1 = t;
        }
        else if (s >= sMax - same) {
            sMax = std::max(sMax, s);
            maxT0 = std::min(maxT0, t);
            maxT1 = std::max(maxT1, t);
        }
        found = true;
    };
    // the points where the line a * s + b * t + c = 0, (a, b) a unit vector, meets the conic
    auto crossings = [&](double a, double b, double c) {
        const double ps = -a * c, pt = -b * c, ds = -b, dt = a;
        const double qa = q[0] * ds * ds + 2 * q[1] * ds * dt + q[2] * dt * dt;
        const double qb = ds * (q[0] * ps + q[1] * pt + q[3]) + dt * (q[1] * ps + q[2] * pt + q[4]);
        const double qc = quadric(ps, pt);
        if (std::abs(qa) < 1e-12) {
            if (std::abs(qb) > 1e-15) {
                const double l = -qc / (2 * qb);
                consider(ps + l * ds, pt + l * dt);
            }
            return;
        }
        double disc = qb * qb - qa * qc;
        if (disc < 0) {
            if (disc < -1e-12 * (qb * qb + std::abs(qa * qc))) {
                return;
            }
            disc = 0;
        }
        const double sq = std::sqrt(disc);
        for (const double l : {(-qb - sq) / qa, (-qb + sq) / qa}) {
            consider(ps + l * ds, pt + l * dt);
        }
    };

    for (int i = 0; i < n; i++) {
        crossings(planes[i].a, planes[i].b, planes[i].c);
        for (int j = i + 1; j < n; j++) {
            const double det = planes[i].a * planes[j].b - planes[j].a * planes[i].b;
            if (std::abs(det) > 1e-9) {
                consider(
                    (planes[i].b * planes[j].c - planes[j].b * planes[i].c) / det,
                    (planes[j].a * planes[i].c - planes[i].a * planes[j].c) / det
                );
            }
        }
    }
    // where the conic is tangent to s: dQ/dt = 0
    const double len = std::sqrt(q[1] * q[1] + q[2] * q[2]);
    if (len > 1e-12) {
        crossings(q[1] / len, q[2] / len, q[4] / len);
    }
    tMin = (minT0 + minT1) / 2;
    tMax = (maxT0 + maxT1) / 2;
    return found;
}

// Unit vectors across a unit tool axis, e1, e2 and the axis right handed
void ToolFrame(const float axis[3], float e1[3], float e2[3])
{
    if (std::abs(axis[0]) < 0.9f) {
        e1[0] = 0;
        e1[1] = axis[2];
        e1[2] = -axis[1];
    }
    else {
        e1[0] = -axis[2];
        e1[1] = 0;
        e1[2] = axis[0];
    }
    const float len = std::sqrt(e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]);
    for (int k = 0; k < 3; k++) {
        e1[k] /= len;
    }
    e2[0] = axis[1] * e1[2] - axis[2] * e1[1];
    e2[1] = axis[2] * e1[0] - axis[0] * e1[2];
    e2[2] = axis[0] * e1[1] - axis[1] * e1[0];
}

const DexelStock::BrickRef& SolidBrick()
{
    static const DexelStock::BrickRef solid = [] {
        auto brick = std::make_shared<Brick>();
        brick->frozen = true;
        brick->solid = true;
        return brick;
    }();
    return solid;
}

// Axes across the lines along axis a
inline int AxisU(int a)
{
    return (a + 1) % 3;
}
inline int AxisV(int a)
{
    return (a + 2) % 3;
}

}  // namespace

uint32_t PackNormal(const float n[3], bool cut)
{
    float x = n[0], y = n[1];
    const float z = n[2];
    const float sum = std::abs(x) + std::abs(y) + std::abs(z);
    uint32_t packed = cut ? 1u << 31 : 0;
    if (sum <= 0) {
        return packed | NoNormal;
    }
    x /= sum;
    y /= sum;
    if (z < 0) {
        const float ox = x, oy = y;
        x = (1 - std::abs(oy)) * (ox >= 0 ? 1.0f : -1.0f);
        y = (1 - std::abs(ox)) * (oy >= 0 ? 1.0f : -1.0f);
    }
    // an even scale: 0, -1 and 1 are exact, so faces along the axes stay exact
    const auto u = (uint32_t)((x * 0.5f + 0.5f) * 4094 + 0.5f);
    const auto v = (uint32_t)((y * 0.5f + 0.5f) * 4094 + 0.5f);
    return packed | (v << 12) | u;
}

void UnpackNormal(uint32_t packed, float n[3])
{
    if (!HasNormal(packed)) {
        n[0] = n[1] = 0;
        n[2] = 1;
        return;
    }
    float x = (float)(packed & 4095) / 4094 * 2 - 1;
    float y = (float)((packed >> 12) & 4095) / 4094 * 2 - 1;
    const float z = 1 - std::abs(x) - std::abs(y);
    if (z < 0) {
        const float ox = x, oy = y;
        x = (1 - std::abs(oy)) * (ox >= 0 ? 1.0f : -1.0f);
        y = (1 - std::abs(ox)) * (oy >= 0 ? 1.0f : -1.0f);
    }
    const float len = std::sqrt(x * x + y * y + z * z);
    n[0] = x / len;
    n[1] = y / len;
    n[2] = z / len;
}

// A move ready to be cut
struct DexelStock::Prepared
{
    struct Instance
    {
        float tip[3];
        float axis[3];
        float e1[3], e2[3];  // tool coordinate system, with the axis
        float lo[3], hi[3];  // bounding box
        float top[3];        // end of the cutting part of the axis
        float length;        // of the cutting part
        bool vertical;
        // lines along axis a, across the tool axis: o + s * across[a] with |across[a]|^2 = across2[a]
        float across[3][3];
        float across2[3];

        // Range of s where the line base + s * e_a is inside the bounding cylinder of the tool
        bool CylinderChord(const float base[3], int a, float r, float& s0, float& s1) const
        {
            const float o[3] = {base[0] - tip[0], base[1] - tip[1], base[2] - tip[2]};
            const float ow = o[0] * axis[0] + o[1] * axis[1] + o[2] * axis[2];
            const float op[3] = {o[0] - ow * axis[0], o[1] - ow * axis[1], o[2] - ow * axis[2]};
            const float* dp = across[a];
            const float qa = across2[a];
            const float qb = 2 * (op[0] * dp[0] + op[1] * dp[1] + op[2] * dp[2]);
            const float qc = op[0] * op[0] + op[1] * op[1] + op[2] * op[2] - r * r * 1.0001f;
            float lo = -1e30f, hi = 1e30f;
            if (qa < 1e-8f) {
                if (qc > 0) {
                    return false;
                }
            }
            else {
                const float disc = qb * qb - 4 * qa * qc;
                if (disc < 0) {
                    return false;
                }
                const float sq = std::sqrt(disc);
                lo = (-qb - sq) / (2 * qa);
                hi = (-qb + sq) / (2 * qa);
            }
            // between the planes at the tip and the top
            const float dw = axis[a];
            if (std::abs(dw) < 1e-8f) {
                if (ow < -1e-4f || ow > length + 1e-4f) {
                    return false;
                }
            }
            else {
                float p0 = -ow / dw, p1 = (length - ow) / dw;
                if (p0 > p1) {
                    std::swap(p0, p1);
                }
                lo = std::max(lo, p0);
                hi = std::min(hi, p1);
            }
            // a margin for rounding
            s0 = lo - 1e-4f;
            s1 = hi + 1e-4f;
            return lo <= hi;
        }
    };

    const CutMove* move = nullptr;
    LayoutRegion region {};
    float cell = 1;
    float length = 0;  // cutting length of the tool
    float radius = 0;  // of the cutting part of the tool
    // vertical tool with disk cross sections: pieces of the move, each changing the height by
    // at most a quarter cell
    bool fast = false;
    int pieces = 1;
    // tilted tool with a fixed axis, made of frustums: the material a line loses is found for
    // the whole move at once, frustum by frustum
    bool swept = false;
    float frame[3][3];  // e1, e2 and the axis
    // otherwise: tool instances along the move
    std::vector<Instance> instances;
    std::vector<int> leaves;

    // A range of the line removed by a swept move, and the times of the move when the tool
    // reaches its ends
    struct Chord
    {
        float lo, hi, tLo, tHi;
    };
    // The ranges of the line base + s * e_a the tool of a swept move passes through, in order. A
    // point of the line is in a frustum of the tool at a time t of the move where it is in a
    // conic region of the (s, t) plane between the planes of the ends of the frustum. The region
    // is convex, its extremes in s are its range on the line.
    // Only if material(lo, hi) says there is material in the range [lo, hi] of the line inside
    // the cylinder around the cutting part of the tool.
    template<class F>
    void SweptChords(const float base[3], int a, std::vector<Chord>& out, F material) const
    {
        out.clear();
        const CutterProfile& tool = *move->tool;
        const auto& frustums = tool.Frustums();
        // the line and the motion in tool coordinates, the motion as a unit vector and its length
        double o[3], d[3], m[3];
        for (int k = 0; k < 3; k++) {
            o[k] = 0;
            d[k] = frame[k][a];
            m[k] = 0;
            for (int c = 0; c < 3; c++) {
                o[k] += ((double)base[c] - move->p0[c]) * frame[k][c];
                m[k] += ((double)move->p1[c] - move->p0[c]) * frame[k][c];
            }
        }
        const double travel = std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
        for (int k = 0; k < 3; k++) {
            m[k] = travel > 1e-9 ? m[k] / travel : 0;
        }
        constexpr double tol = 1e-5;
        // the range on the line of the frustum from z0 to z1, the time along the motion as the
        // distance travelled
        auto range = [&](double z0, double z1, double r0, double r1, Chord& chord) {
            const double k = (r1 - r0) / (z1 - z0);
            const double xs = d[0], xt = -m[0], x0 = o[0];
            const double ys = d[1], yt = -m[1], y0 = o[1];
            const double ws = d[2], wt = -m[2], w0 = o[2];
            const double rs = k * ws, rt = k * wt, r = r0 + k * (w0 - z0);
            const double q[6] = {
                xs * xs + ys * ys - rs * rs,
                xs * xt + ys * yt - rs * rt,
                xt * xt + yt * yt - rt * rt,
                x0 * xs + y0 * ys - r * rs,
                x0 * xt + y0 * yt - r * rt,
                x0 * x0 + y0 * y0 - r * r,
            };
            HalfPlane planes[4];
            int n = 0;
            auto add = [&](double pa, double pb, double pc) {
                const double len = std::sqrt(pa * pa + pb * pb);
                if (len < 1e-12) {
                    return pc >= -tol;
                }
                planes[n++] = {pa / len, pb / len, pc / len};
                return true;
            };
            if (!add(ws, wt, w0 - z0) || !add(-ws, -wt, z1 - w0)) {
                return false;
            }
            add(0, 1, 0);
            add(0, -1, travel);
            double sMin, tMin, sMax, tMax;
            if (!ConvexRange(q, planes, n, tol, sMin, tMin, sMax, tMax)) {
                return false;
            }
            chord = {
                (float)sMin,
                (float)sMax,
                travel > 1e-9 ? (float)(tMin / travel) : 0.0f,
                travel > 1e-9 ? (float)(tMax / travel) : 0.0f,
            };
            return true;
        };

        // nothing if the line misses the cylinder around the cutting part, or has no material there
        Chord chord;
        if (frustums.empty() || !range(frustums.front().z0, length, radius, radius, chord)
            || !material(chord.lo, chord.hi)) {
            return;
        }
        // the heights along the tool the line reaches inside the cylinder during the move
        const double wLo = o[2] + std::min(chord.lo * d[2], chord.hi * d[2]) + std::min(0.0, -travel * m[2]) - tol;
        const double wHi = o[2] + std::max(chord.lo * d[2], chord.hi * d[2]) + std::max(0.0, -travel * m[2]) + tol;
        for (const auto& f : frustums) {
            if (f.z0 >= length || f.z0 > wHi) {
                break;
            }
            if (f.z1 < wLo) {
                continue;
            }
            float z1 = f.z1, r1 = f.r1;
            if (z1 > length) {
                r1 = f.r0 + (length - f.z0) * (f.r1 - f.r0) / (f.z1 - f.z0);
                z1 = length;
            }
            if (z1 - f.z0 > 1e-7f && range(f.z0, z1, f.r0, r1, chord)) {
                out.push_back(chord);
            }
        }
        // the union, in order
        std::sort(out.begin(), out.end(), [](const Chord& x, const Chord& y) { return x.lo < y.lo; });
        size_t kept = 0;
        for (size_t i = 0; i < out.size(); i++) {
            if (kept > 0 && out[i].lo <= out[kept - 1].hi + (float)tol) {
                if (out[i].hi > out[kept - 1].hi) {
                    out[kept - 1].hi = out[i].hi;
                    out[kept - 1].tHi = out[i].tHi;
                }
            }
            else {
                out[kept++] = out[i];
            }
        }
        out.resize(kept);
    }
};

struct DexelStock::InstanceRef
{
    const Prepared* prepared;
    const CutterProfile* tool;
    int index;
};

struct DexelStock::Changes
{
    std::vector<int> lines;  // changed lines, in order
    std::vector<uint32_t> start;
    std::vector<Span> spans;
};

struct DexelStock::Scratch
{
    std::vector<const Prepared*> fastMoves;
    std::vector<InstanceRef> refs;
    // per line of the current axis: the fast moves and the instances that can reach it
    std::vector<std::vector<int>> lineMoves, lineRefs;
    std::vector<Span> line, tmp, removal;
    std::vector<Interval> intervals;
    std::vector<float> normals;
    std::vector<int> instances;
    std::vector<Span> out;
    std::vector<Prepared::Chord> chords;
    long work = 0;
};

void DexelStock::Init(std::shared_ptr<const DexelLayout> layout)
{
    mLayout = std::move(layout);
    mBricks.assign(mLayout->LeafCount(), nullptr);
    mDirty.assign(mBricks.size(), 1);
}

void DexelStock::LineRange(int leaf, int axis, float& lo, float& hi) const
{
    const LayoutLeaf& l = mLayout->Leaf(leaf);
    lo = mLayout->Coord(axis, l.origin[axis] - l.step);
    hi = mLayout->Coord(axis, l.origin[axis] + (BrickCells + 1) * l.step);
}

int DexelStock::Line(int leaf, int line, const Span*& out, Span& scratch) const
{
    const Brick* brick = mBricks[leaf].get();
    if (!brick) {
        out = nullptr;
        return 0;
    }
    if (brick->solid) {
        LineRange(leaf, line / BrickAxisLines, scratch.lo, scratch.hi);
        scratch.nlo = scratch.nhi = 0;
        out = &scratch;
        return 1;
    }
    out = brick->spans.data() + brick->start[line];
    return (int)(brick->start[line + 1] - brick->start[line]);
}

void DexelStock::SetMesh(
    const std::vector<float>& xyz,
    const std::vector<uint32_t>& triangles,
    ThreadPool& pool
)
{
    const DexelLayout& layout = *mLayout;
    mTopZ = -1e30f;
    for (size_t i = 2; i < xyz.size(); i += 3) {
        mTopZ = std::max(mTopZ, xyz[i]);
    }

    // triangle bounding boxes and normals
    const int nTriangles = (int)triangles.size() / 3;
    std::vector<float> boxes(6 * (size_t)nTriangles);
    std::vector<float> normals(3 * (size_t)nTriangles);
    for (int t = 0; t < nTriangles; t++) {
        const float* p[3];
        for (int k = 0; k < 3; k++) {
            p[k] = &xyz[3 * triangles[3 * t + k]];
        }
        for (int k = 0; k < 3; k++) {
            boxes[6 * t + k] = std::min({p[0][k], p[1][k], p[2][k]});
            boxes[6 * t + 3 + k] = std::max({p[0][k], p[1][k], p[2][k]});
        }
        const float e1[3] = {p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]};
        const float e2[3] = {p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]};
        float n[3] = {
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        };
        const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        for (int k = 0; k < 3; k++) {
            normals[3 * t + k] = len > 0 ? n[k] / len : 0;
        }
    }

    // triangles by their projection along each axis, in buckets of the size of the finest bricks
    struct Buckets
    {
        float u0, v0, size;
        int nu, nv;
        std::vector<std::vector<int>> lists;
        const std::vector<int>* At(float u, float v) const
        {
            const int iu = (int)std::floor((u - u0) / size), iv = (int)std::floor((v - v0) / size);
            if (iu < 0 || iv < 0 || iu >= nu || iv >= nv) {
                return nullptr;
            }
            return &lists[(size_t)iv * nu + iu];
        }
    };
    Buckets buckets[3];
    for (int a = 0; a < 3; a++) {
        const int u = AxisU(a), v = AxisV(a);
        Buckets& b = buckets[a];
        b.u0 = layout.Coord(u, 0);
        b.v0 = layout.Coord(v, 0);
        const float extentU = layout.Coord(u, layout.GridCells(u)) - b.u0;
        const float extentV = layout.Coord(v, layout.GridCells(v)) - b.v0;
        b.size = std::max(
            {layout.CellSize(layout.MaxLevel()) * BrickCells, extentU / 1024, extentV / 1024}
        );
        b.nu = std::max(1, (int)std::ceil(extentU / b.size));
        b.nv = std::max(1, (int)std::ceil(extentV / b.size));
        b.lists.assign((size_t)b.nu * b.nv, {});
        for (int t = 0; t < nTriangles; t++) {
            const float* box = &boxes[6 * t];
            const int iu0 = std::max(0, (int)std::floor((box[u] - b.u0) / b.size));
            const int iu1 = std::min(b.nu - 1, (int)std::floor((box[3 + u] - b.u0) / b.size));
            const int iv0 = std::max(0, (int)std::floor((box[v] - b.v0) / b.size));
            const int iv1 = std::min(b.nv - 1, (int)std::floor((box[3 + v] - b.v0) / b.size));
            for (int iv = iv0; iv <= iv1; iv++) {
                for (int iu = iu0; iu <= iu1; iu++) {
                    b.lists[(size_t)iv * b.nu + iu].push_back(t);
                }
            }
        }
    }

    // material along a line, clipped to [tlo, thi]
    struct Hit
    {
        float t;
        int winding;
        int triangle;
    };
    auto castLine = [&](int a, float pu, float pv, float tlo, float thi, std::vector<Hit>& hits, std::vector<Span>& out) {
        out.clear();
        hits.clear();
        const std::vector<int>* list = buckets[a].At(pu, pv);
        if (!list) {
            return;
        }
        const int u = AxisU(a), v = AxisV(a);
        for (int t : *list) {
            const float* box = &boxes[6 * t];
            if (pu < box[u] || pu > box[3 + u] || pv < box[v] || pv > box[3 + v]) {
                continue;
            }
            const float* p[3];
            for (int k = 0; k < 3; k++) {
                p[k] = &xyz[3 * triangles[3 * t + k]];
            }
            double area = (double)(p[1][u] - p[0][u]) * (p[2][v] - p[0][v])
                - (double)(p[1][v] - p[0][v]) * (p[2][u] - p[0][u]);
            if (std::abs(area) < 1e-12) {
                continue;  // parallel to the line
            }
            // going along the axis, a triangle facing backwards enters the material
            const int winding = area < 0 ? 1 : -1;
            if (area < 0) {
                std::swap(p[1], p[2]);
                area = -area;
            }
            double w[3];
            bool inside = true;
            for (int k = 0; k < 3 && inside; k++) {
                const float* e0 = p[(k + 1) % 3];
                const float* e1 = p[(k + 2) % 3];
                // relative to the line, so the reversed edge of the neighboring triangle gives
                // exactly the negated value
                const double ax = e0[u] - pu, ay = e0[v] - pv;
                const double bx = e1[u] - pu, by = e1[v] - pv;
                w[k] = ax * by - ay * bx;
                // top left rule, so lines through shared edges hit one triangle
                const bool topLeft = (e0[v] == e1[v] && e1[u] < e0[u]) || e1[v] < e0[v];
                inside = w[k] > 0 || (w[k] == 0 && topLeft);
            }
            if (!inside) {
                continue;
            }
            // relative to a corner, so triangles perpendicular to the line give exactly their
            // position
            const double z = p[0][a] + (w[1] * (p[1][a] - p[0][a]) + w[2] * (p[2][a] - p[0][a])) / area;
            hits.push_back({(float)z, winding, t});
        }
        std::sort(hits.begin(), hits.end(), [](const Hit& x, const Hit& y) {
            if (x.t != y.t) {
                return x.t < y.t;
            }
            if (x.winding != y.winding) {
                return x.winding < y.winding;
            }
            return x.triangle < y.triangle;
        });
        auto normalOf = [&](const Hit& hit, bool exit) {
            float n[3];
            for (int k = 0; k < 3; k++) {
                n[k] = normals[3 * hit.triangle + k];
            }
            if ((n[a] > 0) != exit) {
                for (float& c : n) {
                    c = -c;
                }
            }
            return PackNormal(n, false);
        };
        int count = 0;
        const Hit* start = nullptr;
        for (const Hit& hit : hits) {
            const int before = count;
            count += hit.winding;
            if (before <= 0 && count > 0) {
                start = &hit;
            }
            else if (before > 0 && count <= 0 && start) {
                Span span {start->t, hit.t, normalOf(*start, false), normalOf(hit, true)};
                if (!out.empty() && span.lo - out.back().hi < MinThickness) {
                    out.back().hi = span.hi;
                    out.back().nhi = span.nhi;
                }
                else if (span.hi - span.lo > MinThickness) {
                    out.push_back(span);
                }
            }
        }
        // clip
        size_t n = 0;
        for (const Span& span : out) {
            Span clipped = span;
            if (clipped.lo < tlo) {
                clipped.lo = tlo;
                clipped.nlo = 0;
            }
            if (clipped.hi > thi) {
                clipped.hi = thi;
                clipped.nhi = 0;
            }
            if (clipped.hi - clipped.lo > MinThickness) {
                out[n++] = clipped;
            }
        }
        out.resize(n);
    };

    const int workers = pool.Size();
    const int leaves = layout.LeafCount();
    pool.Run([&](int worker) {
        std::vector<Hit> hits;
        std::vector<Span> spans;
        for (int leaf = worker; leaf < leaves; leaf += workers) {
            const LayoutLeaf& l = layout.Leaf(leaf);
            float lo[3], hi[3];
            for (int k = 0; k < 3; k++) {
                lo[k] = layout.Coord(k, l.origin[k] - l.step);
                hi[k] = layout.Coord(k, l.origin[k] + (BrickCells + 1) * l.step);
            }
            // completely inside or outside if no triangle comes close
            bool near = false;
            const Buckets& zb = buckets[2];
            const int iu0 = std::max(0, (int)std::floor((lo[0] - zb.u0) / zb.size));
            const int iu1 = std::min(zb.nu - 1, (int)std::floor((hi[0] - zb.u0) / zb.size));
            const int iv0 = std::max(0, (int)std::floor((lo[1] - zb.v0) / zb.size));
            const int iv1 = std::min(zb.nv - 1, (int)std::floor((hi[1] - zb.v0) / zb.size));
            for (int iv = iv0; iv <= iv1 && !near; iv++) {
                for (int iu = iu0; iu <= iu1 && !near; iu++) {
                    for (int t : zb.lists[(size_t)iv * zb.nu + iu]) {
                        const float* box = &boxes[6 * t];
                        if (box[0] <= hi[0] && box[3] >= lo[0] && box[1] <= hi[1] && box[4] >= lo[1]
                            && box[2] <= hi[2] && box[5] >= lo[2]) {
                            near = true;
                            break;
                        }
                    }
                }
            }
            if (!near) {
                const int mid = BrickCells / 2;
                castLine(
                    2,
                    layout.Coord(0, l.origin[0] + mid * l.step),
                    layout.Coord(1, l.origin[1] + mid * l.step),
                    lo[2],
                    hi[2],
                    hits,
                    spans
                );
                const float z = layout.Coord(2, l.origin[2] + mid * l.step);
                bool inside = false;
                for (const Span& span : spans) {
                    inside |= span.lo <= z && z < span.hi;
                }
                mBricks[leaf] = inside ? SolidBrick() : nullptr;
                mDirty[leaf] = 1;
                continue;
            }

            auto brick = std::make_shared<Brick>();
            bool full = true;
            for (int a = 0; a < 3; a++) {
                const int u = AxisU(a), v = AxisV(a);
                for (int j = 0; j < BrickLines; j++) {
                    for (int i = 0; i < BrickLines; i++) {
                        const int line = a * BrickAxisLines + j * BrickLines + i;
                        brick->start[line] = (uint32_t)brick->spans.size();
                        castLine(
                            a,
                            layout.Coord(u, l.origin[u] + i * l.step),
                            layout.Coord(v, l.origin[v] + j * l.step),
                            lo[a],
                            hi[a],
                            hits,
                            spans
                        );
                        full &= spans.size() == 1 && spans[0].lo == lo[a] && spans[0].hi == hi[a];
                        brick->spans.insert(brick->spans.end(), spans.begin(), spans.end());
                    }
                }
            }
            brick->start[BrickLineCount] = (uint32_t)brick->spans.size();
            if (brick->spans.empty()) {
                mBricks[leaf] = nullptr;
            }
            else if (full) {
                mBricks[leaf] = SolidBrick();
            }
            else {
                mBricks[leaf] = brick;
            }
            mDirty[leaf] = 1;
        }
    });
}

void DexelStock::Freeze()
{
    // frozen bricks may be read by other threads, only bricks owned by this stock are written
    for (auto& brick : mBricks) {
        if (brick && !brick->frozen) {
            brick->frozen = true;
        }
    }
}

void DexelStock::Restore(const Snapshot& snapshot)
{
    for (size_t i = 0; i < mBricks.size(); i++) {
        if (mBricks[i] != snapshot[i]) {
            mDirty[i] = 1;
        }
    }
    mBricks = snapshot;
}

std::vector<int> DexelStock::TakeDirtyLeaves()
{
    std::vector<int> dirty;
    for (size_t i = 0; i < mDirty.size(); i++) {
        if (mDirty[i]) {
            dirty.push_back((int)i);
            mDirty[i] = 0;
        }
    }
    return dirty;
}

long DexelStock::TakeNewBrickBytes()
{
    return mNewBrickBytes.exchange(0);
}

float DexelStock::CuttingLength(const CutMove& move) const
{
    const float h = move.tool->Height();
    const float axisZ = std::min(move.a0[2], move.a1[2]);
    if (axisZ < 0.05f) {
        return h;  // (almost) horizontal tools reach the material with their whole length
    }
    const float tipZ = std::min(move.p0[2], move.p1[2]);
    const float side = move.tool->Radius() * std::sqrt(std::max(0.0f, 1 - axisZ * axisZ));
    return std::clamp((mTopZ - tipZ + side) / axisZ, 0.0f, h);
}

void DexelStock::Prepare(const CutMove& move, Prepared& p) const
{
    const CutterProfile& tool = *move.tool;
    p.move = &move;
    p.length = CuttingLength(move);
    // the part of the tool that can reach the material, often narrower than the shank
    const float r = tool.RadiusBelow(p.length);
    p.radius = r;
    p.instances.clear();
    p.leaves.clear();
    const bool vertical = IsVertical(move.a0) && IsVertical(move.a1);
    p.fast = vertical && tool.IsRadiallyMonotonic();
    p.swept = !vertical && Sweeps(move);

    LayoutRegion& region = p.region;
    if (vertical) {
        std::copy(move.p0, move.p0 + 3, region.a);
        std::copy(move.p1, move.p1 + 3, region.b);
        region.radius = r;
        region.vertical = true;
        region.zlo = std::min(move.p0[2], move.p1[2]);
        region.zhi = std::max(move.p0[2], move.p1[2]) + p.length;
        region.UpdateBounds();
    }
    else {
        // bounding box of the tool at both ends of the move, see the instances below
        float lo[3], hi[3];
        for (int k = 0; k < 3; k++) {
            lo[k] = std::min({move.p0[k], move.p1[k], move.p0[k] + move.a0[k] * p.length, move.p1[k] + move.a1[k] * p.length}) - r;
            hi[k] = std::max({move.p0[k], move.p1[k], move.p0[k] + move.a0[k] * p.length, move.p1[k] + move.a1[k] * p.length}) + r;
        }
        // a turning axis sweeps outside of the box of its ends
        float turn = 0;
        for (int k = 0; k < 3; k++) {
            turn = std::max(turn, std::abs(move.a1[k] - move.a0[k]));
        }
        const float bulge = turn * turn * p.length / 4;
        float half2 = 0;
        for (int k = 0; k < 3; k++) {
            lo[k] -= bulge;
            hi[k] += bulge;
            region.a[k] = region.b[k] = (lo[k] + hi[k]) / 2;
            half2 += (hi[k] - lo[k]) * (hi[k] - lo[k]) / 4;
        }
        region.radius = std::sqrt(half2);
        region.vertical = false;
        region.UpdateBounds();
        std::copy(lo, lo + 3, region.lo);
        std::copy(hi, hi + 3, region.hi);
    }
    if (p.length <= 0 && std::min(move.p0[2], move.p1[2]) > mTopZ) {
        return;  // above the material
    }
    const int level = mLayout->MaxLevelIn(region);
    if (level < 0) {
        return;
    }
    p.cell = mLayout->CellSize(level);

    if (p.fast) {
        const float dz = std::abs(move.p1[2] - move.p0[2]);
        const float dxy = std::hypot(move.p1[0] - move.p0[0], move.p1[1] - move.p0[1]);
        const float tolerance = p.cell * 0.25f;
        p.pieces = (dxy > 1e-6f && dz > tolerance) ? (int)std::ceil(dz / tolerance) : 1;
    }
    else if (p.swept) {
        float* axis = p.frame[2];
        const float len = std::sqrt(move.a0[0] * move.a0[0] + move.a0[1] * move.a0[1] + move.a0[2] * move.a0[2]);
        for (int k = 0; k < 3; k++) {
            axis[k] = move.a0[k] / len;
        }
        ToolFrame(axis, p.frame[0], p.frame[1]);
    }
    else {
        // tool instances, close enough that neither the tip nor the top of the cutting part of
        // the tool moves more than half a cell between them
        float dist = 0;
        for (int k = 0; k < 3; k++) {
            const float tip = move.p1[k] - move.p0[k];
            const float top = tip + (move.a1[k] - move.a0[k]) * p.length;
            dist = std::max({dist, std::abs(tip), std::abs(top)});
        }
        // A smooth tool leaves ridges of d^2 / (8 * radius) between instances d apart: at most
        // this fraction of a cell. Sharp edges leave steps, the instances are half a cell apart.
        constexpr float InstanceRidge = 0.01f;
        float spacing = p.cell * 0.5f;
        const float smooth = tool.SmoothRadius(p.length);
        if (smooth > 0) {
            spacing = std::max(spacing, std::sqrt(8 * smooth * InstanceRidge * p.cell));
        }
        const int steps = std::max(1, (int)std::ceil(dist / spacing));
        p.instances.resize(steps + 1);
        for (int i = 0; i <= steps; i++) {
            Prepared::Instance& inst = p.instances[i];
            const float t = (float)i / steps;
            for (int k = 0; k < 3; k++) {
                inst.tip[k] = move.p0[k] + (move.p1[k] - move.p0[k]) * t;
                inst.axis[k] = move.a0[k] + (move.a1[k] - move.a0[k]) * t;
            }
            float* axis = inst.axis;
            const float len = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
            for (int k = 0; k < 3; k++) {
                axis[k] /= len;
            }
            inst.vertical = IsVertical(axis);
            ToolFrame(axis, inst.e1, inst.e2);
            // only the part of the tool below the top of the material can cut
            float h = tool.Height();
            const float side = r * std::sqrt(std::max(0.0f, 1 - axis[2] * axis[2]));
            if (axis[2] > 1e-6f) {
                h = std::clamp((mTopZ - inst.tip[2] + side) / axis[2], 0.0f, h);
            }
            inst.length = h;
            for (int a = 0; a < 3; a++) {
                inst.across2[a] = 0;
                for (int k = 0; k < 3; k++) {
                    inst.across[a][k] = (k == a ? 1.0f : 0.0f) - axis[a] * axis[k];
                    inst.across2[a] += inst.across[a][k] * inst.across[a][k];
                }
            }
            for (int k = 0; k < 3; k++) {
                inst.top[k] = inst.tip[k] + axis[k] * h;
                inst.lo[k] = std::min(inst.tip[k], inst.top[k]) - r;
                inst.hi[k] = std::max(inst.tip[k], inst.top[k]) + r;
            }
        }
    }
    mLayout->LeavesIn(region, p.leaves);
}

bool DexelStock::Sweeps(const CutMove& move)
{
    // a tilted tool turning no more than the rounding of the axes
    bool turning = false;
    for (int k = 0; k < 3; k++) {
        turning |= std::abs(move.a1[k] - move.a0[k]) > 1e-6f;
    }
    return !(IsVertical(move.a0) && IsVertical(move.a1)) && !turning && !move.tool->Frustums().empty();
}

long DexelStock::EstimateWork(const CutMove& move) const
{
    Prepared p;
    Prepare(move, p);
    if (p.leaves.empty()) {
        return 1;
    }
    auto faces = [&](const float* lo, const float* hi) {
        const float dx = hi[0] - lo[0], dy = hi[1] - lo[1], dz = hi[2] - lo[2];
        return (dx * dy + dy * dz + dz * dx) / (p.cell * p.cell);
    };
    double work = 0;
    if (p.fast) {
        work = faces(p.region.lo, p.region.hi) * p.pieces;
    }
    else if (p.swept) {
        // the lines through the region, each with the frustums of the tool, about as costly as
        // a tool instance each
        work = faces(p.region.lo, p.region.hi) * (8.0 + p.move->tool->Frustums().size());
    }
    else {
        for (const auto& inst : p.instances) {
            work += faces(inst.lo, inst.hi);
        }
    }
    return (long)work + 1;
}

long DexelStock::Cut(const std::vector<CutMove>& moves, ThreadPool& pool, const std::atomic<bool>* cancel)
{
    if (moves.empty()) {
        return 0;
    }
    const int workers = pool.Size();
    std::vector<Prepared> prepared(moves.size());
    pool.Run([&](int worker) {
        for (size_t i = worker; i < moves.size(); i += workers) {
            Prepare(moves[i], prepared[i]);
        }
    });
    // the moves touching each brick, every brick is cut once with all of them
    std::vector<std::pair<int, int>> touches;
    for (int i = 0; i < (int)prepared.size(); i++) {
        for (int leaf : prepared[i].leaves) {
            touches.emplace_back(leaf, i);
        }
    }
    std::sort(touches.begin(), touches.end());
    // the bricks, those with the most moves first, taken by the threads as they become free
    std::vector<std::pair<size_t, size_t>> groups;
    for (size_t first = 0; first < touches.size();) {
        size_t last = first;
        while (last < touches.size() && touches[last].first == touches[first].first) {
            last++;
        }
        if (mBricks[touches[first].first]) {
            groups.emplace_back(first, last);
        }
        first = last;
    }
    std::stable_sort(groups.begin(), groups.end(), [](const auto& x, const auto& y) {
        return x.second - x.first > y.second - y.first;
    });
    // every brick in parts by axis and lines, so heavy bricks are shared by threads, then the
    // changes of the parts are put together
    constexpr int Parts = 3;
    const size_t tasks = groups.size() * 3 * Parts;
    std::vector<Changes> changes(tasks);
    std::atomic<size_t> next {0};
    std::vector<long> work(workers, 0);
    pool.Run([&](int worker) {
        Scratch scratch;
        std::vector<const Prepared*> list;
        while (!(cancel && *cancel)) {
            const size_t task = next++;
            if (task >= tasks) {
                break;
            }
            const auto& group = groups[task / (3 * Parts)];
            const int axis = (int)(task / Parts % 3), part = (int)(task % Parts);
            list.clear();
            for (size_t t = group.first; t < group.second; t++) {
                list.push_back(&prepared[touches[t].second]);
            }
            CutBrick(
                touches[group.first].first,
                list.data(),
                (int)list.size(),
                axis,
                part * BrickLines / Parts,
                (part + 1) * BrickLines / Parts,
                scratch,
                changes[task]
            );
        }
        work[worker] = scratch.work;
    });
    if (cancel && *cancel) {
        return 0;
    }
    next = 0;
    pool.Run([&](int) {
        Scratch scratch;
        while (true) {
            const size_t g = next++;
            if (g >= groups.size()) {
                break;
            }
            Commit(touches[groups[g].first].first, &changes[g * 3 * Parts], 3 * Parts, scratch);
        }
    });
    long total = 0;
    for (long w : work) {
        total += w;
    }
    return total;
}

void DexelStock::CutBrick(
    int leaf,
    const Prepared* const* moves,
    int count,
    int axis,
    int lineFrom,
    int lineTo,
    Scratch& s,
    Changes& changes
)
{
    const DexelLayout& layout = *mLayout;
    const LayoutLeaf& l = layout.Leaf(leaf);
    changes.lines.clear();
    changes.start.clear();
    changes.spans.clear();

    // tool instances near the brick, by tool, and the moves of vertical tools
    s.fastMoves.clear();
    s.refs.clear();
    float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
    for (int m = 0; m < count; m++) {
        const Prepared& p = *moves[m];
        if (p.fast || p.swept) {
            s.fastMoves.push_back(&p);
        }
        else {
            for (int i = 0; i < (int)p.instances.size(); i++) {
                const auto& inst = p.instances[i];
                bool overlaps = true;
                for (int k = 0; k < 3; k++) {
                    overlaps &= inst.hi[k] >= layout.Coord(k, l.origin[k] - l.step)
                        && inst.lo[k] <= layout.Coord(k, l.origin[k] + (BrickCells + 1) * l.step);
                }
                if (overlaps) {
                    s.refs.push_back({&p, p.move->tool, i});
                }
            }
            if (s.refs.empty() || s.refs.back().prepared != &p) {
                continue;
            }
        }
        for (int k = 0; k < 3; k++) {
            lo[k] = std::min(lo[k], p.region.lo[k]);
            hi[k] = std::max(hi[k], p.region.hi[k]);
        }
    }
    if (s.fastMoves.empty() && s.refs.empty()) {
        return;
    }
    std::stable_sort(s.refs.begin(), s.refs.end(), [](const InstanceRef& x, const InstanceRef& y) {
        return x.tool < y.tool;
    });

    // The current line: its spans in the brick until a move changes it, then a copy in
    // s.line, so lines no move changes are not copied
    const Span* lineData = nullptr;
    int lineSize = 0;
    bool copied = false;
    // subtract a removal of a move from the current line, its faces get the operation of the move
    auto apply = [&](Span* removal, int nr, const CutMove& move) {
        if (nr == 0 || lineSize == 0 || removal[nr - 1].hi <= lineData[0].lo
            || removal[0].lo >= lineData[lineSize - 1].hi) {
            return;
        }
        for (int k = 0; k < nr; k++) {
            removal[k].nlo = WithOperationTag(removal[k].nlo, move.operationTag);
            removal[k].nhi = WithOperationTag(removal[k].nhi, move.operationTag);
        }
        s.tmp.resize(lineSize + nr);
        s.tmp.resize(Subtract(lineData, lineSize, removal, nr, s.tmp.data()));
        s.line.swap(s.tmp);
        lineData = s.line.data();
        lineSize = (int)s.line.size();
        copied = true;
    };

    Span solid;
    s.lineMoves.resize(BrickAxisLines);
    s.lineRefs.resize(BrickAxisLines);
    for (int a = axis; a == axis; a++) {
        const int u = AxisU(a), v = AxisV(a);
        float tlo, thi;
        LineRange(leaf, a, tlo, thi);
        if (thi < lo[a] || tlo > hi[a]) {
            continue;
        }
        // the lines of the brick inside a box across them
        float cu[BrickLines], cv[BrickLines];
        for (int k = 0; k < BrickLines; k++) {
            cu[k] = layout.Coord(u, l.origin[u] + k * l.step);
            cv[k] = layout.Coord(v, l.origin[v] + k * l.step);
        }
        auto bucket = [&](const float* blo, const float* bhi, int item, std::vector<std::vector<int>>& lists) {
            if (bhi[a] < tlo || blo[a] > thi) {
                return;
            }
            const int i0 = (int)(std::lower_bound(cu, cu + BrickLines, blo[u]) - cu);
            const int i1 = (int)(std::upper_bound(cu, cu + BrickLines, bhi[u]) - cu);
            const int j0 = (int)(std::lower_bound(cv, cv + BrickLines, blo[v]) - cv);
            const int j1 = (int)(std::upper_bound(cv, cv + BrickLines, bhi[v]) - cv);
            for (int j = j0; j < j1; j++) {
                for (int i = i0; i < i1; i++) {
                    lists[j * BrickLines + i].push_back(item);
                }
            }
        };
        for (auto& list : s.lineMoves) {
            list.clear();
        }
        for (auto& list : s.lineRefs) {
            list.clear();
        }
        for (int m = 0; m < (int)s.fastMoves.size(); m++) {
            bucket(s.fastMoves[m]->region.lo, s.fastMoves[m]->region.hi, m, s.lineMoves);
        }
        for (int ref = 0; ref < (int)s.refs.size(); ref++) {
            const auto& inst = s.refs[ref].prepared->instances[s.refs[ref].index];
            bucket(inst.lo, inst.hi, ref, s.lineRefs);
        }
        for (int j = lineFrom; j < lineTo; j++) {
            const float pv = layout.Coord(v, l.origin[v] + j * l.step);
            if (pv < lo[v] || pv > hi[v]) {
                continue;
            }
            for (int i = 0; i < BrickLines; i++) {
                const float pu = layout.Coord(u, l.origin[u] + i * l.step);
                if (pu < lo[u] || pu > hi[u]) {
                    continue;
                }
                const int line = a * BrickAxisLines + j * BrickLines + i;
                const auto& lineMoves = s.lineMoves[j * BrickLines + i];
                const auto& lineRefs = s.lineRefs[j * BrickLines + i];
                if (lineMoves.empty() && lineRefs.empty()) {
                    continue;
                }
                const Span* spans;
                const int n = Line(leaf, line, spans, solid);
                if (n == 0 || spans[n - 1].hi < lo[a] || spans[0].lo > hi[a]) {
                    continue;
                }
                s.work++;
                lineData = spans;
                lineSize = n;
                copied = false;
                // world position of the line: coordinate 'pu' along u, 'pv' along v
                float base[3];
                base[a] = 0;
                base[u] = pu;
                base[v] = pv;

                // vertical tools, move by move
                for (int m : lineMoves) {
                    const Prepared& p = *s.fastMoves[m];
                    const LayoutRegion& region = p.region;
                    if (pu < region.lo[u] || pu > region.hi[u] || pv < region.lo[v] || pv > region.hi[v]
                        || lineSize == 0 || lineData[lineSize - 1].hi < region.lo[a] || lineData[0].lo > region.hi[a]) {
                        continue;
                    }
                    const CutMove& move = *p.move;
                    const CutterProfile& tool = *move.tool;
                    const float r = p.radius;
                    if (p.swept) {
                        p.SweptChords(base, a, s.chords, [&](float lo, float hi) {
                            return MaterialIn(lineData, lineSize, lo, hi);
                        });
                        s.removal.clear();
                        for (const auto& chord : s.chords) {
                            if (!MaterialIn(lineData, lineSize, chord.lo, chord.hi)) {
                                continue;
                            }
                            // normals from the tool where it reaches the ends that become ends of
                            // the material
                            uint32_t normal[2] = {0, 0};
                            const float ends[2] = {chord.lo, chord.hi}, times[2] = {chord.tLo, chord.tHi};
                            for (int e = 0; e < 2; e++) {
                                if (!InsideMaterial(lineData, lineSize, ends[e])) {
                                    continue;
                                }
                                float origin[3], dir[3];
                                for (int k = 0; k < 3; k++) {
                                    origin[k] = 0;
                                    for (int c = 0; c < 3; c++) {
                                        const float tip = move.p0[c] + times[e] * (move.p1[c] - move.p0[c]);
                                        origin[k] += (base[c] - tip) * p.frame[k][c];
                                    }
                                    dir[k] = p.frame[k][a];
                                }
                                tool.LineIntervals(origin, dir, s.intervals, &s.normals, p.cell);
                                int best = -1;
                                float distance = 1e30f;
                                for (int k = 0; k < (int)s.intervals.size(); k++) {
                                    const float dk = std::abs((e == 0 ? s.intervals[k].lo : s.intervals[k].hi) - ends[e]);
                                    if (dk < distance) {
                                        distance = dk;
                                        best = k;
                                    }
                                }
                                if (best >= 0) {
                                    const float* nt = &s.normals[6 * best + 3 * e];
                                    float nw[3];
                                    for (int c = 0; c < 3; c++) {
                                        nw[c] = nt[0] * p.frame[0][c] + nt[1] * p.frame[1][c] + nt[2] * p.frame[2][c];
                                    }
                                    normal[e] = ToolNormal3(nw);
                                }
                            }
                            s.removal.push_back({chord.lo, chord.hi, normal[0], normal[1]});
                        }
                        apply(s.removal.data(), (int)s.removal.size(), move);
                        continue;
                    }
                    if (a == 2) {
        // vertical line through (x, y)
        const float cx = base[0], cy = base[1];
        // The tool is nowhere lower on the line than its lowest point at the closest approach of
        // the move, at the lowest height of the move: nothing to do if that is above the
        // material, like for the passes beside the one finishing the surface there.
        if (tool.HasSingleAxialIntervals()) {
            const float mx = move.p1[0] - move.p0[0], my = move.p1[1] - move.p0[1];
            const float len2 = mx * mx + my * my;
            const float tc = len2 > 0
                ? std::clamp(((cx - move.p0[0]) * mx + (cy - move.p0[1]) * my) / len2, 0.0f, 1.0f)
                : 0.0f;
            const float ex = cx - move.p0[0] - tc * mx, ey = cy - move.p0[1] - tc * my;
            const float dc = std::sqrt(ex * ex + ey * ey);
            const Interval* iv;
            if (dc > r || tool.AxialIntervals(dc, iv) == 0
                || lineSize == 0 || std::min(move.p0[2], move.p1[2]) + iv[0].lo >= lineData[lineSize - 1].hi) {
                continue;
            }
        }
        // only the pieces of the part of the move within the radius of the line
        int firstPiece = 0, lastPiece = p.pieces - 1;
        if (p.pieces > 1) {
            const float mx = move.p1[0] - move.p0[0], my = move.p1[1] - move.p0[1];
            const float ox = move.p0[0] - cx, oy = move.p0[1] - cy;
            const float qa = mx * mx + my * my, qb = 2 * (ox * mx + oy * my), qc = ox * ox + oy * oy - r * r;
            const float disc = qb * qb - 4 * qa * qc;
            if (qa <= 0 || disc < 0) {
                continue;  // never within the radius
            }
            const float sq = std::sqrt(disc);
            const float ta = (-qb - sq) / (2 * qa), tb = (-qb + sq) / (2 * qa);
            if (tb < 0 || ta > 1) {
                continue;
            }
            firstPiece = std::clamp((int)std::floor(ta * p.pieces) - 1, 0, p.pieces - 1);
            lastPiece = std::clamp((int)std::ceil(tb * p.pieces), 0, p.pieces - 1);
        }
        for (int piece = firstPiece; piece <= lastPiece; piece++) {
            float pa[3], pb[3];
            for (int k = 0; k < 3; k++) {
                pa[k] = move.p0[k] + (move.p1[k] - move.p0[k]) * piece / p.pieces;
                pb[k] = move.p0[k] + (move.p1[k] - move.p0[k]) * (piece + 1) / p.pieces;
            }
            const float dx = pb[0] - pa[0], dy = pb[1] - pa[1];
            const float len2 = dx * dx + dy * dy;
            float t = 0;
            if (len2 > 0) {
                t = std::clamp(((cx - pa[0]) * dx + (cy - pa[1]) * dy) / len2, 0.0f, 1.0f);
            }
            const float ex = cx - pa[0] - t * dx, ey = cy - pa[1] - t * dy;
            const float d = std::sqrt(ex * ex + ey * ey);
            if (d > r) {
                continue;
            }
            const float zmin = std::min(pa[2], pb[2]), zmax = std::max(pa[2], pb[2]);
            // With one solid interval per line, the removed material is one interval
            // from the lowest bottom to the highest top of the tool along the move,
            // taken from the tool at the start, the end and the closest approach.
            // Otherwise the solid of the closest approach is extended over the
            // height range of the move, which can remove slightly more.
            Span removal[64];
            int nr = 0;
            const Interval* iv;
            const ProfileNormal* pn;
            if (tool.HasSingleAxialIntervals() && zmin != zmax) {
                float lo = 1e30f, hi = -1e30f;
                // the normals of the lowest and highest point, packed once
                ProfileNormal pnLo {}, pnHi {};
                float hLo[2] = {0, 0}, hHi[2] = {0, 0};
                for (const float tc : {0.0f, t, 1.0f}) {
                    const float hx = cx - pa[0] - tc * dx, hy = cy - pa[1] - tc * dy;
                    const float dc = std::sqrt(hx * hx + hy * hy);
                    if (tool.AxialIntervals(dc, iv, pn) == 1) {
                        const float z = pa[2] + tc * (pb[2] - pa[2]);
                        const float ux = dc > 1e-9f ? hx / dc : 0, uy = dc > 1e-9f ? hy / dc : 0;
                        if (iv[0].lo + z < lo) {
                            lo = iv[0].lo + z;
                            pnLo = pn[0];
                            hLo[0] = ux;
                            hLo[1] = uy;
                        }
                        if (iv[0].hi + z > hi) {
                            hi = iv[0].hi + z;
                            pnHi = pn[1];
                            hHi[0] = ux;
                            hHi[1] = uy;
                        }
                    }
                }
                // the normals of the ends that become ends of the material only
                if (lo < hi && lineSize > 0 && lo < lineData[lineSize - 1].hi && hi > lineData[0].lo) {
                    const uint32_t nlo = InsideMaterial(lineData, lineSize, lo) ? ToolNormal(pnLo, hLo[0], hLo[1]) : 0;
                    const uint32_t nhi = InsideMaterial(lineData, lineSize, hi) ? ToolNormal(pnHi, hHi[0], hHi[1]) : 0;
                    removal[nr++] = {lo, hi, nlo, nhi};
                }
            }
            else {
                const float hx = cx - pa[0] - t * dx, hy = cy - pa[1] - t * dy;
                const float ux = d > 1e-9f ? hx / d : 0, uy = d > 1e-9f ? hy / d : 0;
                const int n = std::min(64, tool.AxialIntervals(d, iv, pn));
                for (int k = 0; k < n; k++) {
                    const Span sp {
                        iv[k].lo + zmin,
                        iv[k].hi + zmax,
                        ToolNormal(pn[2 * k], ux, uy),
                        ToolNormal(pn[2 * k + 1], ux, uy),
                    };
                    if (nr > 0 && sp.lo <= removal[nr - 1].hi) {
                        if (sp.hi > removal[nr - 1].hi) {
                            removal[nr - 1].hi = sp.hi;
                            removal[nr - 1].nhi = sp.nhi;
                        }
                    }
                    else {
                        removal[nr++] = sp;
                    }
                }
            }
            apply(removal, nr, move);
        }
                    }
                    else {
        // horizontal line at height zl, along x (a = 0) or y (a = 1): the tool's
        // slices at that height sweep the hull of the slices at the ends of the parts
        // of the move between heights of profile vertices, where their radius
        // changes linearly
        const float zl = base[2];
        const int across = a == 0 ? 1 : 0;
        const float w = base[across];
        const float z0 = move.p0[2], dz = move.p1[2] - move.p0[2];
        // The tool is nowhere lower at the line than its lowest point at the horizontal
        // distance of the line from the move, at the lowest height of the move: lines below
        // that, inside the material under the cut surface, keep their material.
        if (tool.HasSingleAxialIntervals()) {
            const float w0 = move.p0[across] - w, w1 = move.p1[across] - w;
            const float dc = (w0 < 0) != (w1 < 0) ? 0.0f : std::min(std::abs(w0), std::abs(w1));
            const Interval* iv;
            if (dc > r || tool.AxialIntervals(dc, iv) == 0 || std::min(z0, z0 + dz) + iv[0].lo > zl) {
                continue;
            }
        }
        // nothing to do without material where the tool passes along the line
        if (!MaterialIn(lineData, lineSize, std::min(move.p0[a], move.p1[a]) - r, std::max(move.p0[a], move.p1[a]) + r)) {
            continue;
        }
        // range of the move where the line is at the height of the cutting part
        float t0 = 0, t1 = 1;
        if (std::abs(dz) < 1e-9f) {
            if (zl - z0 < 0 || zl - z0 > p.length) {
                t1 = -1;
            }
        }
        else {
            t0 = (zl - z0 - p.length) / dz;
            t1 = (zl - z0) / dz;
            if (t0 > t1) {
                std::swap(t0, t1);
            }
            t0 = std::max(t0, 0.0f);
            t1 = std::min(t1, 1.0f);
        }
        if (t0 > t1) {
            // not at this height
        }
        else {
            float breaks[66];
            int nb = 0;
            breaks[nb++] = t0;
            if (std::abs(dz) >= 1e-9f) {
                for (float h : tool.VertexHeights()) {
                    const float t = (zl - z0 - h) / dz;
                    if (t > t0 && t < t1 && nb < 65) {
                        breaks[nb++] = t;
                    }
                }
            }
            breaks[nb++] = t1;
            std::sort(breaks, breaks + nb);
            float lo = 1e30f, hi = -1e30f, tLo = 0, tHi = 0;
            for (int b = 0; b + 1 < nb; b++) {
                const float ta = breaks[b], tb = breaks[b + 1];
                if (tb < ta || (b > 0 && tb == ta)) {
                    continue;
                }
                const float ra = std::max(0.0f, tool.SliceRadius(zl - z0 - ta * dz));
                const float rb = std::max(0.0f, tool.SliceRadius(zl - z0 - tb * dz));
                const float ca[2] = {
                    move.p0[a] + ta * (move.p1[a] - move.p0[a]),
                    move.p0[across] + ta * (move.p1[across] - move.p0[across]),
                };
                const float cb[2] = {
                    move.p0[a] + tb * (move.p1[a] - move.p0[a]),
                    move.p0[across] + tb * (move.p1[across] - move.p0[across]),
                };
                float s0, s1, l0 = 0, l1 = 0;
                if (!HullChord(ca, ra, cb, rb, w, s0, s1, l0, l1)) {
                    continue;
                }
                if (s0 < lo) {
                    lo = s0;
                    tLo = ta + l0 * (tb - ta);
                }
                if (s1 > hi) {
                    hi = s1;
                    tHi = ta + l1 * (tb - ta);
                }
            }
            if (lo < hi && MaterialIn(lineData, lineSize, lo, hi)) {
                // normals from the tool at the ends that become ends of the material
                uint32_t normal[2] = {0, 0};
                const float ends[2] = {lo, hi}, times[2] = {tLo, tHi};
                for (int e = 0; e < 2; e++) {
                    if (!InsideMaterial(lineData, lineSize, ends[e])) {
                        continue;
                    }
                    const float t = times[e];
                    ProfileNormal pn {};
                    tool.SliceRadius(zl - z0 - t * dz, pn);
                    float h[2] = {
                        ends[e] - (move.p0[a] + t * (move.p1[a] - move.p0[a])),
                        w - (move.p0[across] + t * (move.p1[across] - move.p0[across])),
                    };
                    const float len = std::hypot(h[0], h[1]);
                    if (len > 1e-9f) {
                        h[0] /= len;
                        h[1] /= len;
                    }
                    else {
                        h[0] = e == 0 ? -1.0f : 1.0f;
                        h[1] = 0;
                    }
                    float hxy[2];
                    hxy[a] = h[0];
                    hxy[across] = h[1];
                    normal[e] = ToolNormal(pn, hxy[0], hxy[1]);
                }
                Span removal {lo, hi, normal[0], normal[1]};
                apply(&removal, 1, move);
            }
        }
                    }
                }

                // tool instances, skipped if there is no material left in the bounding cylinder
                // of the tool
                for (size_t g = 0; g < lineRefs.size();) {
                    const CutterProfile& tool = *s.refs[lineRefs[g]].tool;
                    size_t end = g;
                    while (end < lineRefs.size() && s.refs[lineRefs[end]].tool == &tool) {
                        end++;
                    }
                    for (size_t k = g; k < end; k++) {
                        const InstanceRef& ref = s.refs[lineRefs[k]];
                        const Prepared& p = *ref.prepared;
                        const float r = p.radius;
                        const auto& inst = p.instances[ref.index];
                        if (base[u] < inst.lo[u] || base[u] > inst.hi[u] || base[v] < inst.lo[v]
                            || base[v] > inst.hi[v]) {
                            continue;
                        }
                        // the line misses the tool if it is farther than the radius from the
                        // axis, measured across the line
                        {
                            const float tu = inst.tip[u] - base[u], tv = inst.tip[v] - base[v];
                            const float du = inst.top[u] - inst.tip[u], dv = inst.top[v] - inst.tip[v];
                            const float len2 = du * du + dv * dv;
                            float t = 0;
                            if (len2 > 0) {
                                t = std::clamp(-(tu * du + tv * dv) / len2, 0.0f, 1.0f);
                            }
                            const float cu = tu + t * du, cv = tv + t * dv;
                            if (cu * cu + cv * cv > r * r * 1.0001f + 1e-8f) {
                                continue;
                            }
                        }
                        {
                            float b0, b1;
                            if (!inst.CylinderChord(base, a, r, b0, b1) || !MaterialIn(lineData, lineSize, b0, b1)) {
                                continue;
                            }
                        }
                        s.removal.clear();
                        if (inst.vertical && a == 2) {
                            const float hx = base[0] - inst.tip[0], hy = base[1] - inst.tip[1];
                            const float d = std::hypot(hx, hy);
                            const Interval* iv;
                            const ProfileNormal* pn;
                            const int na = tool.AxialIntervals(d, iv, pn);
                            const float ux = d > 1e-9f ? hx / d : 0, uy = d > 1e-9f ? hy / d : 0;
                            for (int k = 0; k < na; k++) {
                                s.removal.push_back({
                                    iv[k].lo + inst.tip[2],
                                    iv[k].hi + inst.tip[2],
                                    ToolNormal(pn[2 * k], ux, uy),
                                    ToolNormal(pn[2 * k + 1], ux, uy),
                                });
                            }
                        }
                        else {
                            // line in tool coordinates
                            const float o[3] = {
                                base[0] - inst.tip[0],
                                base[1] - inst.tip[1],
                                base[2] - inst.tip[2],
                            };
                            const float origin[3] = {
                                o[0] * inst.e1[0] + o[1] * inst.e1[1] + o[2] * inst.e1[2],
                                o[0] * inst.e2[0] + o[1] * inst.e2[1] + o[2] * inst.e2[2],
                                o[0] * inst.axis[0] + o[1] * inst.axis[1] + o[2] * inst.axis[2],
                            };
                            const float dir[3] = {inst.e1[a], inst.e2[a], inst.axis[a]};
                            tool.LineIntervals(origin, dir, s.intervals, &s.normals, p.cell);
                            for (size_t k = 0; k < s.intervals.size(); k++) {
                                // back to world coordinates
                                float nw[2][3];
                                for (int e = 0; e < 2; e++) {
                                    const float* nt = &s.normals[6 * k + 3 * e];
                                    for (int c = 0; c < 3; c++) {
                                        nw[e][c] = nt[0] * inst.e1[c] + nt[1] * inst.e2[c] + nt[2] * inst.axis[c];
                                    }
                                }
                                s.removal.push_back({
                                    s.intervals[k].lo,
                                    s.intervals[k].hi,
                                    ToolNormal3(nw[0]),
                                    ToolNormal3(nw[1]),
                                });
                            }
                        }
                        apply(s.removal.data(), (int)s.removal.size(), *p.move);
                    }
                    g = end;
                }

                // keep the line if it changed
                bool same = !copied || (int)s.line.size() == n;
                for (int k = 0; copied && same && k < n; k++) {
                    same = s.line[k].lo == spans[k].lo && s.line[k].hi == spans[k].hi;
                }
                if (!same) {
                    changes.lines.push_back(line);
                    changes.start.push_back((uint32_t)changes.spans.size());
                    changes.spans.insert(changes.spans.end(), s.line.begin(), s.line.end());
                }
            }
        }
    }
}

void DexelStock::Commit(int leaf, Changes* parts, int count, Scratch& s)
{
    bool any = false;
    for (int p = 0; p < count; p++) {
        parts[p].start.push_back((uint32_t)parts[p].spans.size());
        any |= !parts[p].lines.empty();
    }
    if (!any) {
        return;
    }
    const BrickRef& old = mBricks[leaf];
    s.out.clear();
    uint32_t start[BrickLineCount + 1];
    // the parts are in the order of their lines
    int part = 0;
    size_t next = 0;
    Span solid;
    for (int line = 0; line < BrickLineCount; line++) {
        start[line] = (uint32_t)s.out.size();
        while (part < count && next >= parts[part].lines.size()) {
            part++;
            next = 0;
        }
        if (part < count && parts[part].lines[next] == line) {
            const Changes& c = parts[part];
            s.out.insert(s.out.end(), c.spans.begin() + c.start[next], c.spans.begin() + c.start[next + 1]);
            next++;
            continue;
        }
        const Span* spans;
        const int n = Line(leaf, line, spans, solid);
        s.out.insert(s.out.end(), spans, spans + n);
    }
    start[BrickLineCount] = (uint32_t)s.out.size();
    mDirty[leaf] = 1;
    if (s.out.empty()) {
        mBricks[leaf] = nullptr;
        return;
    }
    if (old->frozen) {
        auto copy = std::make_shared<Brick>();
        copy->spans = s.out;
        std::memcpy(copy->start, start, sizeof(start));
        mNewBrickBytes += (long)(sizeof(Brick) + copy->spans.size() * sizeof(Span));
        mBricks[leaf] = std::move(copy);
        return;
    }
    old->spans.swap(s.out);
    std::memcpy(old->start, start, sizeof(start));
}

}  // namespace CAMSimulator
