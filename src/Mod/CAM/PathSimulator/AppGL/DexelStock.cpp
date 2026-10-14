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

// Union of sorted, disjoint span lists into 'acc', keeping the normals of the outer ends
void Unite(std::vector<Span>& acc, const std::vector<Span>& add, std::vector<Span>& out)
{
    out.clear();
    size_t i = 0, j = 0;
    while (i < acc.size() || j < add.size()) {
        const Span& next = j >= add.size() || (i < acc.size() && acc[i].lo <= add[j].lo) ? acc[i++] : add[j++];
        if (!out.empty() && next.lo <= out.back().hi) {
            if (next.hi > out.back().hi) {
                out.back().hi = next.hi;
                out.back().nhi = next.nhi;
            }
        }
        else {
            out.push_back(next);
        }
    }
    acc.swap(out);
}

// Range of s where the line base + s * e_a is inside the cylinder of radius r around the segment
// from 'tip' along the unit vector 'axis' of 'length'. Returns false if there is none.
bool CylinderChord(const float base[3], int a, const float tip[3], const float axis[3], float length, float r, float& s0, float& s1)
{
    // relative to the tip, the line is o + s * d with d the unit vector of axis a
    const double o[3] = {(double)base[0] - tip[0], (double)base[1] - tip[1], (double)base[2] - tip[2]};
    const double ow = o[0] * axis[0] + o[1] * axis[1] + o[2] * axis[2];
    const double dw = axis[a];
    // across the axis: o_perp + s * d_perp
    double op[3], dp[3];
    for (int k = 0; k < 3; k++) {
        op[k] = o[k] - ow * axis[k];
        dp[k] = (k == a ? 1.0 : 0.0) - dw * axis[k];
    }
    const double qa = dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2];
    const double qb = 2 * (op[0] * dp[0] + op[1] * dp[1] + op[2] * dp[2]);
    const double qc = op[0] * op[0] + op[1] * op[1] + op[2] * op[2] - (double)r * r;
    double lo = -1e30, hi = 1e30;
    if (qa < 1e-12) {
        if (qc > 0) {
            return false;
        }
    }
    else {
        const double disc = qb * qb - 4 * qa * qc;
        if (disc < 0) {
            return false;
        }
        const double sq = std::sqrt(disc);
        lo = (-qb - sq) / (2 * qa);
        hi = (-qb + sq) / (2 * qa);
    }
    // between the planes at the tip and the top
    if (std::abs(dw) < 1e-12) {
        if (ow < 0 || ow > length) {
            return false;
        }
    }
    else {
        double p0 = -ow / dw, p1 = (length - ow) / dw;
        if (p0 > p1) {
            std::swap(p0, p1);
        }
        lo = std::max(lo, p0);
        hi = std::min(hi, p1);
    }
    s0 = (float)lo;
    s1 = (float)hi;
    return lo <= hi;
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
    const auto u = (uint32_t)((x * 0.5f + 0.5f) * 32767 + 0.5f);
    const auto v = (uint32_t)((y * 0.5f + 0.5f) * 32767 + 0.5f);
    return packed | (v << 15) | u;
}

void UnpackNormal(uint32_t packed, float n[3])
{
    if (!HasNormal(packed)) {
        n[0] = n[1] = 0;
        n[2] = 1;
        return;
    }
    float x = (float)(packed & 32767) / 32767 * 2 - 1;
    float y = (float)((packed >> 15) & 32767) / 32767 * 2 - 1;
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
    };

    const CutMove* move = nullptr;
    LayoutRegion region {};
    float cell = 1;
    float length = 0;  // cutting length of the tool
    // vertical tool with disk cross sections: pieces of the move, each changing the height by
    // at most a quarter cell
    bool fast = false;
    int pieces = 1;
    // otherwise: tool instances along the move
    std::vector<Instance> instances;
    std::vector<int> leaves;
};

struct DexelStock::Scratch
{
    std::vector<Span> line, tmp, removal, united, merged;
    std::vector<Interval> intervals;
    std::vector<float> normals;
    std::vector<int> instances;
    // changed lines of the current brick
    std::vector<int> changed;
    std::vector<uint32_t> changedStart;
    std::vector<Span> changedSpans;
    std::vector<Span> out;
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
    const float r = tool.Radius();
    p.move = &move;
    p.length = CuttingLength(move);
    p.instances.clear();
    p.leaves.clear();
    const bool vertical = IsVertical(move.a0) && IsVertical(move.a1);
    p.fast = vertical && tool.IsRadiallyMonotonic();

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
    else {
        // tool instances, close enough that neither the tip nor the top of the cutting part of
        // the tool moves more than half a cell between them
        float dist = 0;
        for (int k = 0; k < 3; k++) {
            const float tip = move.p1[k] - move.p0[k];
            const float top = tip + (move.a1[k] - move.a0[k]) * p.length;
            dist = std::max({dist, std::abs(tip), std::abs(top)});
        }
        const int steps = std::max(1, (int)std::ceil(dist / (p.cell * 0.5f)));
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
            float* e1 = inst.e1;
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
            const float elen = std::sqrt(e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]);
            for (int k = 0; k < 3; k++) {
                e1[k] /= elen;
            }
            inst.e2[0] = axis[1] * e1[2] - axis[2] * e1[1];
            inst.e2[1] = axis[2] * e1[0] - axis[0] * e1[2];
            inst.e2[2] = axis[0] * e1[1] - axis[1] * e1[0];
            // only the part of the tool below the top of the material can cut
            float h = tool.Height();
            const float side = r * std::sqrt(std::max(0.0f, 1 - axis[2] * axis[2]));
            if (axis[2] > 1e-6f) {
                h = std::clamp((mTopZ - inst.tip[2] + side) / axis[2], 0.0f, h);
            }
            inst.length = h;
            for (int k = 0; k < 3; k++) {
                inst.top[k] = inst.tip[k] + axis[k] * h;
                inst.lo[k] = std::min(inst.tip[k], inst.top[k]) - r;
                inst.hi[k] = std::max(inst.tip[k], inst.top[k]) + r;
            }
        }
    }
    mLayout->LeavesIn(region, p.leaves);
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
    std::vector<long> work(workers, 0);
    pool.Run([&](int worker) {
        Scratch scratch;
        for (const Prepared& p : prepared) {
            if (cancel && *cancel) {
                break;
            }
            for (int leaf : p.leaves) {
                if (leaf % workers == worker && mBricks[leaf]) {
                    CutBrick(leaf, p, scratch);
                }
            }
        }
        work[worker] = scratch.work;
    });
    long total = 0;
    for (long w : work) {
        total += w;
    }
    return total;
}

void DexelStock::CutBrick(int leaf, const Prepared& p, Scratch& s)
{
    const DexelLayout& layout = *mLayout;
    const LayoutLeaf& l = layout.Leaf(leaf);
    const CutMove& move = *p.move;
    const CutterProfile& tool = *move.tool;
    const float r = tool.Radius();
    const LayoutRegion& region = p.region;
    s.changed.clear();
    s.changedStart.clear();
    s.changedSpans.clear();

    // instances near the brick
    if (!p.fast) {
        s.instances.clear();
        for (int i = 0; i < (int)p.instances.size(); i++) {
            const auto& inst = p.instances[i];
            bool overlaps = true;
            for (int k = 0; k < 3; k++) {
                overlaps &= inst.hi[k] >= layout.Coord(k, l.origin[k] - l.step)
                    && inst.lo[k] <= layout.Coord(k, l.origin[k] + (BrickCells + 1) * l.step);
            }
            if (overlaps) {
                s.instances.push_back(i);
            }
        }
        if (s.instances.empty()) {
            return;
        }
    }

    // subtract a removal from the current line
    auto apply = [&](const Span* removal, int nr) {
        if (nr == 0 || s.line.empty() || removal[nr - 1].hi <= s.line.front().lo
            || removal[0].lo >= s.line.back().hi) {
            return;
        }
        s.tmp.resize(s.line.size() + nr);
        s.tmp.resize(Subtract(s.line.data(), (int)s.line.size(), removal, nr, s.tmp.data()));
        s.line.swap(s.tmp);
    };

    Span solid;
    for (int a = 0; a < 3; a++) {
        const int u = AxisU(a), v = AxisV(a);
        float tlo, thi;
        LineRange(leaf, a, tlo, thi);
        if (thi < region.lo[a] || tlo > region.hi[a]) {
            continue;
        }
        for (int j = 0; j < BrickLines; j++) {
            const float pv = layout.Coord(v, l.origin[v] + j * l.step);
            if (pv < region.lo[v] || pv > region.hi[v]) {
                continue;
            }
            for (int i = 0; i < BrickLines; i++) {
                const float pu = layout.Coord(u, l.origin[u] + i * l.step);
                if (pu < region.lo[u] || pu > region.hi[u]) {
                    continue;
                }
                const int line = a * BrickAxisLines + j * BrickLines + i;
                const Span* spans;
                const int n = Line(leaf, line, spans, solid);
                if (n == 0 || spans[n - 1].hi < region.lo[a] || spans[0].lo > region.hi[a]) {
                    continue;
                }
                s.work++;
                s.line.assign(spans, spans + n);
                // world position of the line: coordinate 'pu' along u, 'pv' along v
                float base[3];
                base[a] = 0;
                base[u] = pu;
                base[v] = pv;

                if (p.fast && a == 2) {
                    // vertical line through (x, y)
                    const float cx = base[0], cy = base[1];
                    for (int piece = 0; piece < p.pieces; piece++) {
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
                        const float d = std::hypot(cx - pa[0] - t * dx, cy - pa[1] - t * dy);
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
                            uint32_t nlo = 0, nhi = 0;
                            for (const float tc : {0.0f, t, 1.0f}) {
                                const float hx = cx - pa[0] - tc * dx, hy = cy - pa[1] - tc * dy;
                                const float dc = std::hypot(hx, hy);
                                if (tool.AxialIntervals(dc, iv, pn) == 1) {
                                    const float z = pa[2] + tc * (pb[2] - pa[2]);
                                    const float ux = dc > 1e-9f ? hx / dc : 0, uy = dc > 1e-9f ? hy / dc : 0;
                                    if (iv[0].lo + z < lo) {
                                        lo = iv[0].lo + z;
                                        nlo = ToolNormal(pn[0], ux, uy);
                                    }
                                    if (iv[0].hi + z > hi) {
                                        hi = iv[0].hi + z;
                                        nhi = ToolNormal(pn[1], ux, uy);
                                    }
                                }
                            }
                            if (lo < hi) {
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
                        apply(removal, nr);
                    }
                }
                else if (p.fast) {
                    // horizontal line at height zl, along x (a = 0) or y (a = 1): the tool's
                    // slices at that height sweep the hull of the slices at the ends of the parts
                    // of the move between heights of profile vertices, where their radius
                    // changes linearly
                    const float zl = base[2];
                    const int across = a == 0 ? 1 : 0;
                    const float w = base[across];
                    const float z0 = move.p0[2], dz = move.p1[2] - move.p0[2];
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
                            ProfileNormal unused;
                            const float ra = std::max(0.0f, tool.SliceRadius(zl - z0 - ta * dz, unused));
                            const float rb = std::max(0.0f, tool.SliceRadius(zl - z0 - tb * dz, unused));
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
                        if (lo < hi) {
                            // normals from the tool at the ends
                            uint32_t normal[2];
                            const float ends[2] = {lo, hi}, times[2] = {tLo, tHi};
                            for (int e = 0; e < 2; e++) {
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
                            const Span removal {lo, hi, normal[0], normal[1]};
                            apply(&removal, 1);
                        }
                    }
                }
                else {
                    // tool instances. The material removed by convex tools is united first, an
                    // instance that can't add to it is skipped.
                    const bool convex = tool.IsConvex();
                    s.united.clear();
                    for (int index : s.instances) {
                        const auto& inst = p.instances[index];
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
                        if (convex && !s.united.empty()) {
                            float b0, b1;
                            if (!CylinderChord(base, a, inst.tip, inst.axis, inst.length, r, b0, b1)) {
                                continue;
                            }
                            bool covered = false;
                            for (const Span& united : s.united) {
                                covered |= united.lo <= b0 && b1 <= united.hi;
                            }
                            if (covered) {
                                continue;
                            }
                        }
                        s.removal.clear();
                        if (inst.vertical && a == 2) {
                            const float hx = base[0] - inst.tip[0], hy = base[1] - inst.tip[1];
                            const float d = std::hypot(hx, hy);
                            const Interval* iv;
                            const ProfileNormal* pn;
                            const int n = tool.AxialIntervals(d, iv, pn);
                            const float ux = d > 1e-9f ? hx / d : 0, uy = d > 1e-9f ? hy / d : 0;
                            for (int k = 0; k < n; k++) {
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
                                    const float* n = &s.normals[6 * k + 3 * e];
                                    for (int c = 0; c < 3; c++) {
                                        nw[e][c] = n[0] * inst.e1[c] + n[1] * inst.e2[c] + n[2] * inst.axis[c];
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
                        if (convex) {
                            Unite(s.united, s.removal, s.merged);
                        }
                        else {
                            apply(s.removal.data(), (int)s.removal.size());
                        }
                    }
                    if (convex) {
                        apply(s.united.data(), (int)s.united.size());
                    }
                }

                // keep the line if it changed
                bool same = (int)s.line.size() == n;
                for (int k = 0; same && k < n; k++) {
                    same = s.line[k].lo == spans[k].lo && s.line[k].hi == spans[k].hi;
                }
                if (!same) {
                    s.changed.push_back(line);
                    s.changedStart.push_back((uint32_t)s.changedSpans.size());
                    s.changedSpans.insert(s.changedSpans.end(), s.line.begin(), s.line.end());
                }
            }
        }
    }
    Commit(leaf, s);
}

void DexelStock::Commit(int leaf, Scratch& s)
{
    if (s.changed.empty()) {
        return;
    }
    s.changedStart.push_back((uint32_t)s.changedSpans.size());
    const BrickRef& old = mBricks[leaf];
    s.out.clear();
    uint32_t start[BrickLineCount + 1];
    size_t next = 0;
    Span solid;
    for (int line = 0; line < BrickLineCount; line++) {
        start[line] = (uint32_t)s.out.size();
        if (next < s.changed.size() && s.changed[next] == line) {
            s.out.insert(
                s.out.end(),
                s.changedSpans.begin() + s.changedStart[next],
                s.changedSpans.begin() + s.changedStart[next + 1]
            );
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
