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

#include "MaterialGrid.h"

#include "ThreadPool.h"

#include <algorithm>
#include <cmath>

namespace CAMSimulator::Material
{

namespace
{

// Run fn(i) for i in [0, n) on all threads of the pool, in chunks taken as threads get free
template<class F>
void ParallelFor(ThreadPool& pool, int n, int chunk, const F& fn)
{
    if (n <= 0) {
        return;
    }
    std::atomic<int> next {0};
    pool.Run([&](int) {
        for (;;) {
            const int begin = next.fetch_add(chunk);
            if (begin >= n) {
                return;
            }
            const int end = std::min(n, begin + chunk);
            for (int i = begin; i < end; i++) {
                fn(i);
            }
        }
    });
}

inline int AxisU(int a)
{
    return (a + 1) % 3;
}
inline int AxisV(int a)
{
    return (a + 2) % 3;
}


}  // namespace

const Grid::BrickRef& Grid::Solid()
{
    static const BrickRef solid = std::make_shared<Brick>();
    return solid;
}

void Grid::Init(std::shared_ptr<const Layout> layout)
{
    mLayout = std::move(layout);
    mBricks.assign(mLayout->LeafCount(), nullptr);
    mDirty.assign(mLayout->LeafCount(), 1);
}

void Grid::LineRange(int leaf, int axis, double& lo, double& hi) const
{
    const Layout::Leaf& l = mLayout->GetLeaf(leaf);
    lo = mLayout->Coord(axis, l.lo[axis] - l.cell);
    hi = mLayout->Coord(axis, l.lo[axis] + (Cells + 1) * l.cell);
}

void Grid::LineOrigin(int leaf, int line, double origin[3]) const
{
    const Layout::Leaf& l = mLayout->GetLeaf(leaf);
    const int axis = line / AxisLines;
    const int rest = line % AxisLines;
    const int i = rest % (Cells + 1), j = rest / (Cells + 1);
    origin[axis] = mLayout->Coord(axis, l.lo[axis]);
    origin[AxisU(axis)] = mLayout->Coord(AxisU(axis), l.lo[AxisU(axis)] + i * l.cell);
    origin[AxisV(axis)] = mLayout->Coord(AxisV(axis), l.lo[AxisV(axis)] + j * l.cell);
}

int Grid::Line(int leaf, int line, const Span*& out, Span& scratch) const
{
    const BrickRef& b = mBricks[leaf];
    if (!b) {
        return 0;
    }
    if (b == Solid()) {
        double lo, hi;
        LineRange(leaf, line / AxisLines, lo, hi);
        scratch = {(float)lo, (float)hi, NoNormal, NoNormal};
        out = &scratch;
        return 1;
    }
    out = b->spans.data() + b->start[line];
    return (int)(b->start[line + 1] - b->start[line]);
}

void Grid::SetMesh(const std::vector<float>& xyz, const std::vector<uint32_t>& triangles, ThreadPool& pool)
{
    const int leaves = mLayout->LeafCount();
    mBricks.assign(leaves, nullptr);
    mDirty.assign(leaves, 1);
    const size_t ntri = triangles.size() / 3;
    if (ntri == 0) {
        return;
    }
    double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
    for (size_t i = 0; i + 2 < xyz.size(); i += 3) {
        for (int k = 0; k < 3; k++) {
            lo[k] = std::min(lo[k], (double)xyz[i + k]);
            hi[k] = std::max(hi[k], (double)xyz[i + k]);
        }
    }
    // per axis, a 2D grid of the triangles projected along it
    struct Grid2D
    {
        double lo[2], cell[2];
        int n[2];
        std::vector<std::vector<uint32_t>> cells;
    };
    Grid2D grids[3];
    const int res = std::clamp((int)std::sqrt((double)ntri), 1, 256);
    for (int a = 0; a < 3; a++) {
        Grid2D& g = grids[a];
        const int ax[2] = {AxisU(a), AxisV(a)};
        for (int k = 0; k < 2; k++) {
            g.lo[k] = lo[ax[k]];
            g.n[k] = res;
            g.cell[k] = std::max(1e-9, (hi[ax[k]] - lo[ax[k]]) / res);
        }
        g.cells.assign((size_t)res * res, {});
        for (size_t t = 0; t < ntri; t++) {
            double tlo[2] = {1e300, 1e300}, thi[2] = {-1e300, -1e300};
            for (int c = 0; c < 3; c++) {
                const float* p = &xyz[3 * triangles[3 * t + c]];
                for (int k = 0; k < 2; k++) {
                    tlo[k] = std::min(tlo[k], (double)p[ax[k]]);
                    thi[k] = std::max(thi[k], (double)p[ax[k]]);
                }
            }
            int i0[2], i1[2];
            for (int k = 0; k < 2; k++) {
                i0[k] = std::clamp((int)std::floor((tlo[k] - g.lo[k]) / g.cell[k]), 0, res - 1);
                i1[k] = std::clamp((int)std::floor((thi[k] - g.lo[k]) / g.cell[k]), 0, res - 1);
            }
            for (int j = i0[1]; j <= i1[1]; j++) {
                for (int i = i0[0]; i <= i1[0]; i++) {
                    g.cells[(size_t)j * res + i].push_back((uint32_t)t);
                }
            }
        }
    }

    ParallelFor(pool, leaves, 16, [&](int leaf) {
        const Layout::Leaf& l = mLayout->GetLeaf(leaf);
        // outside of the box of the mesh: empty
        bool outside = false;
        for (int k = 0; k < 3; k++) {
            outside = outside || mLayout->Coord(k, l.lo[k] + Cells * l.cell) < lo[k]
                || mLayout->Coord(k, l.lo[k]) > hi[k];
        }
        if (outside) {
            return;
        }
        // no triangle near the leaf (with the cell its lines reach beyond it): all inside or
        // all outside, as one of its lines
        bool touched = false;
        {
            double blo[3], bhi[3];
            for (int k = 0; k < 3; k++) {
                blo[k] = mLayout->Coord(k, l.lo[k] - l.cell);
                bhi[k] = mLayout->Coord(k, l.lo[k] + (Cells + 1) * l.cell);
            }
            const Grid2D& g = grids[2];
            const int i0 = std::clamp((int)std::floor((blo[0] - g.lo[0]) / g.cell[0]), 0, g.n[0] - 1);
            const int i1 = std::clamp((int)std::floor((bhi[0] - g.lo[0]) / g.cell[0]), 0, g.n[0] - 1);
            const int j0 = std::clamp((int)std::floor((blo[1] - g.lo[1]) / g.cell[1]), 0, g.n[1] - 1);
            const int j1 = std::clamp((int)std::floor((bhi[1] - g.lo[1]) / g.cell[1]), 0, g.n[1] - 1);
            for (int j = j0; j <= j1 && !touched; j++) {
                for (int i = i0; i <= i1 && !touched; i++) {
                    for (uint32_t t : g.cells[(size_t)j * g.n[0] + i]) {
                        double tlo[3] = {1e300, 1e300, 1e300}, thi[3] = {-1e300, -1e300, -1e300};
                        for (int c = 0; c < 3; c++) {
                            const float* p = &xyz[3 * triangles[3 * t + c]];
                            for (int k = 0; k < 3; k++) {
                                tlo[k] = std::min(tlo[k], (double)p[k]);
                                thi[k] = std::max(thi[k], (double)p[k]);
                            }
                        }
                        bool overlap = true;
                        for (int k = 0; k < 3; k++) {
                            overlap = overlap && tlo[k] <= bhi[k] && thi[k] >= blo[k];
                        }
                        if (overlap) {
                            touched = true;
                            break;
                        }
                    }
                }
            }
        }
        auto brick = std::make_shared<Brick>();
        struct Hit
        {
            double s;
            float n[3];
        };
        std::vector<Hit> hits;
        bool solid = true;
        for (int line = 0; line < (touched ? BrickLines : 1); line++) {
            brick->start[line] = (uint16_t)brick->spans.size();
            const int a = line / AxisLines;
            double origin[3];
            LineOrigin(leaf, line, origin);
            double sLo, sHi;
            LineRange(leaf, a, sLo, sHi);
            const Grid2D& g = grids[a];
            const int ax[2] = {AxisU(a), AxisV(a)};
            const double pu = origin[ax[0]], pv = origin[ax[1]];
            const int ci = (int)std::floor((pu - g.lo[0]) / g.cell[0]);
            const int cj = (int)std::floor((pv - g.lo[1]) / g.cell[1]);
            hits.clear();
            if (ci >= 0 && cj >= 0 && ci < g.n[0] && cj < g.n[1] && pu <= hi[ax[0]] && pv <= hi[ax[1]]) {
                for (uint32_t t : g.cells[(size_t)cj * g.n[0] + ci]) {
                    const float* p[3] = {
                        &xyz[3 * triangles[3 * t]],
                        &xyz[3 * triangles[3 * t + 1]],
                        &xyz[3 * triangles[3 * t + 2]],
                    };
                    // 2D edge functions with a top-left rule, so points on shared edges count once
                    double area = 0;
                    double e[3];
                    for (int c = 0; c < 3; c++) {
                        // in the same order of its ends in both triangles of an edge, so the
                        // two are exact negatives of each other
                        const float* q0 = p[c];
                        const float* q1 = p[(c + 1) % 3];
                        const bool swap = q1[ax[0]] < q0[ax[0]]
                            || (q1[ax[0]] == q0[ax[0]] && q1[ax[1]] < q0[ax[1]]);
                        if (swap) {
                            std::swap(q0, q1);
                        }
                        e[c] = ((double)q1[ax[0]] - q0[ax[0]]) * (pv - q0[ax[1]])
                            - ((double)q1[ax[1]] - q0[ax[1]]) * (pu - q0[ax[0]]);
                        if (swap) {
                            e[c] = -e[c];
                        }
                        area += e[c];
                    }
                    if (area == 0) {
                        continue;
                    }
                    const double sign = area > 0 ? 1 : -1;
                    bool inside = true;
                    for (int c = 0; c < 3 && inside; c++) {
                        const double ec = e[c] * sign;
                        if (ec > 0) {
                            continue;
                        }
                        if (ec < 0) {
                            inside = false;
                            continue;
                        }
                        // on the edge: include top and left edges only
                        const float* q0 = p[c];
                        const float* q1 = p[(c + 1) % 3];
                        const double du = ((double)q1[ax[0]] - q0[ax[0]]) * sign;
                        const double dv = ((double)q1[ax[1]] - q0[ax[1]]) * sign;
                        inside = dv < 0 || (dv == 0 && du > 0);
                    }
                    if (!inside) {
                        continue;
                    }
                    // barycentric: weight of vertex c is the edge function of the opposite edge
                    const double s = (e[1] * p[0][a] + e[2] * p[1][a] + e[0] * p[2][a]) / area;
                    const double e1[3] = {
                        (double)p[1][0] - p[0][0],
                        (double)p[1][1] - p[0][1],
                        (double)p[1][2] - p[0][2],
                    };
                    const double e2[3] = {
                        (double)p[2][0] - p[0][0],
                        (double)p[2][1] - p[0][1],
                        (double)p[2][2] - p[0][2],
                    };
                    double n[3] = {
                        e1[1] * e2[2] - e1[2] * e2[1],
                        e1[2] * e2[0] - e1[0] * e2[2],
                        e1[0] * e2[1] - e1[1] * e2[0],
                    };
                    const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
                    hits.push_back({s, {(float)(n[0] / len), (float)(n[1] / len), (float)(n[2] / len)}});
                }
            }
            std::sort(hits.begin(), hits.end(), [](const Hit& x, const Hit& y) { return x.s < y.s; });
            int spans = 0;
            for (size_t h = 0; h + 1 < hits.size(); h += 2) {
                Hit enter = hits[h], leave = hits[h + 1];
                // material normals point out of the material: against the axis at entry
                if (enter.n[a] > 0) {
                    for (float& c : enter.n) {
                        c = -c;
                    }
                }
                if (leave.n[a] < 0) {
                    for (float& c : leave.n) {
                        c = -c;
                    }
                }
                double s0 = enter.s, s1 = leave.s;
                uint32_t n0 = PackNormal(enter.n), n1 = PackNormal(leave.n);
                if (s1 < sLo || s0 > sHi) {
                    continue;
                }
                if (s0 < sLo) {
                    s0 = sLo;
                    n0 = NoNormal;
                }
                if (s1 > sHi) {
                    s1 = sHi;
                    n1 = NoNormal;
                }
                if (s1 - s0 < 1e-6 || brick->spans.size() >= Brick::MaxSpans) {
                    continue;
                }
                brick->spans.push_back({(float)s0, (float)s1, n0, n1});
                spans++;
                solid = solid && s0 <= sLo && s1 >= sHi;
            }
            solid = solid && spans == 1;
        }
        brick->start[BrickLines] = (uint16_t)brick->spans.size();
        if (brick->spans.empty()) {
            return;
        }
        if (solid) {
            mBricks[leaf] = Solid();
            return;
        }
        if (!touched) {
            return;  // the line of a uniform leaf is all inside or all outside
        }
        mBricks[leaf] = std::move(brick);
    });
}

long Grid::Cut(
    const std::vector<const Sweep*>& sweeps,
    ThreadPool& pool,
    std::vector<std::pair<int, BrickRef>>* undo
)
{
    const int leaves = mLayout->LeafCount();
    const int ns = (int)sweeps.size();
    if (ns == 0) {
        return 0;
    }
    // leaves touched by every sweep
    std::vector<std::vector<int>> touched(ns);
    ParallelFor(pool, ns, 4, [&](int i) {
        const Sweep& s = *sweeps[i];
        double lo[3], hi[3];
        for (int k = 0; k < 3; k++) {
            lo[k] = s.BoxLo()[k] - 1e-6;
            hi[k] = s.BoxHi()[k] + 1e-6;
        }
        std::vector<int> found;
        mLayout->LeavesWhere(
            lo,
            hi,
            [&](const double c[3], double half) { return s.MayTouch(c, half + 1e-6); },
            found
        );
        for (int leaf : found) {
            if (mBricks[leaf]) {
                touched[i].push_back(leaf);
            }
        }
    });
    // sweeps by leaf, in order
    std::vector<int> count(leaves + 1, 0);
    for (int i = 0; i < ns; i++) {
        for (int leaf : touched[i]) {
            count[leaf + 1]++;
        }
    }
    for (int l = 0; l < leaves; l++) {
        count[l + 1] += count[l];
    }
    std::vector<int> bySweep(count[leaves]);
    std::vector<int> fill(count.begin(), count.end() - 1);
    std::vector<int> work;
    for (int i = 0; i < ns; i++) {
        for (int leaf : touched[i]) {
            bySweep[fill[leaf]++] = i;
        }
    }
    for (int l = 0; l < leaves; l++) {
        if (count[l + 1] > count[l]) {
            work.push_back(l);
        }
    }
    // most sweeps first, for the balance of the threads
    std::sort(work.begin(), work.end(), [&](int a, int b) {
        return count[a + 1] - count[a] > count[b + 1] - count[b];
    });

    std::atomic<long> done {0};
    std::vector<std::pair<int, BrickRef>> replaced(undo ? work.size() : 0, {-1, nullptr});
    ParallelFor(pool, (int)work.size(), 1, [&](int w) {
        const int leaf = work[w];
        done += CutLeaf(
            leaf,
            sweeps,
            bySweep.data() + count[leaf],
            count[leaf + 1] - count[leaf],
            undo ? &replaced[w] : nullptr
        );
    });
    if (undo) {
        for (auto& r : replaced) {
            if (r.first >= 0) {
                undo->push_back(std::move(r));
            }
        }
    }
    return done;
}

namespace
{

// A span being cut: its ends are packed normals, or removal ends whose normals are computed when
// they remain at the end
struct WorkSpan
{
    double lo, hi;
    uint32_t nlo, nhi;
    int plo, phi;  // pending removal ends, -1 for none
};

struct PendingEnd
{
    int sweep;
    RemovalEnd end;
};

}  // namespace

long Grid::CutLeaf(
    int leaf,
    const std::vector<const Sweep*>& sweeps,
    const int* list,
    int n,
    std::pair<int, BrickRef>* undo
)
{
    thread_local std::vector<WorkSpan> lines[BrickLines];
    thread_local std::vector<WorkSpan> next;
    thread_local std::vector<Removal> removals;
    thread_local std::vector<PendingEnd> pending;
    pending.clear();

    const Layout::Leaf& lf = mLayout->GetLeaf(leaf);
    const double cell = lf.cell * mLayout->Unit();
    double corner[3];
    for (int k = 0; k < 3; k++) {
        corner[k] = mLayout->Coord(k, lf.lo[k]);
    }
    for (int line = 0; line < BrickLines; line++) {
        Span scratch;
        const Span* spans;
        const int nspans = Line(leaf, line, spans, scratch);
        lines[line].clear();
        for (int i = 0; i < nspans; i++) {
            lines[line].push_back({spans[i].lo, spans[i].hi, spans[i].nlo, spans[i].nhi, -1, -1});
        }
    }

    // sweep by sweep in the order of the path, the lines in its box: later sweeps see the
    // material earlier ones left
    bool changed = false;
    long lineWork = 0;
    for (int k = 0; k < n; k++) {
        const int si = list[k];
        const Sweep& sw = *sweeps[si];
        for (int a = 0; a < 3; a++) {
            const int au = AxisU(a), av = AxisV(a);
            const int i0 = std::max(0, (int)std::ceil((sw.BoxLo()[au] - corner[au]) / cell - 1e-9));
            const int i1 = std::min(Cells, (int)std::floor((sw.BoxHi()[au] - corner[au]) / cell + 1e-9));
            const int j0 = std::max(0, (int)std::ceil((sw.BoxLo()[av] - corner[av]) / cell - 1e-9));
            const int j1 = std::min(Cells, (int)std::floor((sw.BoxHi()[av] - corner[av]) / cell + 1e-9));
            double origin[3];
            origin[a] = corner[a];
            for (int j = j0; j <= j1; j++) {
                origin[av] = mLayout->Coord(av, lf.lo[av] + j * lf.cell);
                for (int i = i0; i <= i1; i++) {
                    std::vector<WorkSpan>& spans = lines[LineIndex(a, i, j)];
                    if (spans.empty()) {
                        continue;
                    }
                    origin[au] = mLayout->Coord(au, lf.lo[au] + i * lf.cell);
                    removals.clear();
                    lineWork++;
                    if (sw.Line(a, origin, spans.front().lo, spans.back().hi, removals) == 0) {
                        continue;
                    }
                    // subtract
                    next.clear();
                    size_t r = 0;
                    bool lineChanged = false;
                    for (const WorkSpan& span : spans) {
                        WorkSpan cur = span;
                        // removals only touching the span leave it as it is
                        while (r < removals.size() && removals[r].hi.s <= cur.lo + 1e-7) {
                            r++;
                        }
                        size_t q = r;
                        bool alive = true;
                        while (q < removals.size() && removals[q].lo.s < cur.hi - 1e-7) {
                            const Removal& m = removals[q];
                            lineChanged = true;
                            if (m.lo.s > cur.lo + 1e-6) {
                                // material before the removal remains, ending at its start
                                pending.push_back({si, m.lo});
                                next.push_back({cur.lo, m.lo.s, cur.nlo, 0, cur.plo, (int)pending.size() - 1});
                            }
                            if (m.hi.s < cur.hi) {
                                pending.push_back({si, m.hi});
                                cur.lo = m.hi.s;
                                cur.nlo = 0;
                                cur.plo = (int)pending.size() - 1;
                                q++;
                            }
                            else {
                                alive = false;
                                break;
                            }
                        }
                        if (alive && cur.hi - cur.lo > 1e-6) {
                            next.push_back(cur);
                        }
                    }
                    if (lineChanged) {
                        spans.swap(next);
                        changed = true;
                    }
                }
            }
        }
    }
    if (!changed) {
        return lineWork;
    }

    // the new brick, with the normals of the ends made by the sweeps
    auto brick = std::make_shared<Brick>();
    auto pack = [&](int line, int p) {
        const PendingEnd& e = pending[p];
        const Sweep& sw = *sweeps[e.sweep];
        double origin[3];
        LineOrigin(leaf, line, origin);
        float nrm[3];
        uint32_t packed = sw.Normal(line / AxisLines, origin, e.end, nrm) ? PackNormal(nrm) : NoNormal;
        return packed | CutFlag | ((uint32_t)sw.GetMotion().tag << TagShift);
    };
    for (int line = 0; line < BrickLines; line++) {
        brick->start[line] = (uint16_t)brick->spans.size();
        for (const WorkSpan& span : lines[line]) {
            if (brick->spans.size() >= Brick::MaxSpans) {
                break;  // finer than a brick can hold, never in practice
            }
            const uint32_t nlo = span.plo >= 0 ? pack(line, span.plo) : span.nlo;
            const uint32_t nhi = span.phi >= 0 ? pack(line, span.phi) : span.nhi;
            brick->spans.push_back({(float)span.lo, (float)span.hi, nlo, nhi});
        }
    }
    brick->start[BrickLines] = (uint16_t)brick->spans.size();
    if (undo) {
        *undo = {leaf, mBricks[leaf]};
    }
    if (brick->spans.empty()) {
        mBricks[leaf] = nullptr;
    }
    else {
        brick->spans.shrink_to_fit();
        mBricks[leaf] = std::move(brick);
    }
    mDirty[leaf] = 1;
    return lineWork;
}

void Grid::Restore(const Snapshot& snapshot)
{
    for (size_t i = 0; i < mBricks.size(); i++) {
        if (mBricks[i] != snapshot[i]) {
            mBricks[i] = snapshot[i];
            mDirty[i] = 1;
        }
    }
}

std::vector<int> Grid::TakeDirty()
{
    std::vector<int> out;
    for (size_t i = 0; i < mDirty.size(); i++) {
        if (mDirty[i]) {
            out.push_back((int)i);
            mDirty[i] = 0;
        }
    }
    return out;
}

void Grid::MarkAllDirty()
{
    std::fill(mDirty.begin(), mDirty.end(), 1);
}

}  // namespace CAMSimulator::Material
