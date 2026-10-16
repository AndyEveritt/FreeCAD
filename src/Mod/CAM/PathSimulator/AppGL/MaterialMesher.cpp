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

#include "MaterialMesher.h"

#include "ThreadPool.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <unordered_map>

namespace CAMSimulator::Material
{

namespace
{

constexpr int CellCount = Cells * Cells * Cells;
constexpr int EdgesPerAxis = AxisLines * Cells;
constexpr int EdgeCount = 3 * EdgesPerAxis;
// normals of merged cubes agree within about 8 degrees
constexpr float MergeCos = 0.99f;
// vertices with normals further apart than about 25 degrees are sharp
constexpr float SharpCos = 0.9f;

inline int AxisU(int a)
{
    return (a + 1) % 3;
}
inline int AxisV(int a)
{
    return (a + 2) % 3;
}
inline int CellIndex(int x, int y, int z)
{
    return (z * Cells + y) * Cells + x;
}
// edge along axis a on the line through grid point (i, j) of the other axes, from t to t + 1
inline int EdgeIndex(int a, int i, int j, int t)
{
    return a * EdgesPerAxis + (j * (Cells + 1) + i) * Cells + t;
}

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

// A surface point on an edge
struct Crossing
{
    float p[3];
    float n[3];
    bool hasNormal;
    bool cut;
    bool startInside;  // the edge starts inside the material: the surface faces along it
    uint8_t tag;
};

// Inside the material at a grid point of a leaf, from its dexel along Z
bool Inside(const Grid& grid, int leaf, int i, int j, int k)
{
    const Layout& L = grid.GetLayout();
    const Layout::Leaf& lf = L.GetLeaf(leaf);
    Span scratch;
    const Span* spans;
    const int n = grid.Line(leaf, LineIndex(2, i, j), spans, scratch);
    const float z = (float)L.Coord(2, lf.lo[2] + k * lf.cell);
    for (int s = 0; s < n; s++) {
        if (z < spans[s].lo) {
            return false;
        }
        if (z < spans[s].hi) {
            return true;
        }
    }
    return false;
}

// The surface point on an edge of a leaf whose ends differ in inside, nearest to its middle
void FindCrossing(const Grid& grid, int leaf, int a, int i, int j, int t, bool startInside, Crossing& c)
{
    const Layout& L = grid.GetLayout();
    const Layout::Leaf& lf = L.GetLeaf(leaf);
    const int u = AxisU(a), v = AxisV(a);
    const double s0 = L.Coord(a, lf.lo[a] + t * lf.cell), s1 = L.Coord(a, lf.lo[a] + (t + 1) * lf.cell);
    const double mid = 0.5 * (s0 + s1);
    Span scratch;
    const Span* spans;
    const int n = grid.Line(leaf, LineIndex(a, i, j), spans, scratch);
    double best = 1e30, pos = mid;
    uint32_t packed = NoNormal;
    for (int s = 0; s < n; s++) {
        // leaving the material at hi, entering at lo
        const double e = startInside ? spans[s].hi : spans[s].lo;
        const uint32_t pn = startInside ? spans[s].nhi : spans[s].nlo;
        if (e >= s0 && e <= s1 && std::abs(e - mid) < best) {
            best = std::abs(e - mid);
            pos = e;
            packed = pn;
        }
    }
    c.p[a] = (float)pos;
    c.p[u] = (float)L.Coord(u, lf.lo[u] + i * lf.cell);
    c.p[v] = (float)L.Coord(v, lf.lo[v] + j * lf.cell);
    c.hasNormal = HasNormal(packed) && best < 1e30;
    if (c.hasNormal) {
        UnpackNormal(packed, c.n);
    }
    else {
        c.n[0] = c.n[1] = c.n[2] = 0;
        c.n[a] = startInside ? 1.0f : -1.0f;
    }
    c.cut = IsCut(packed);
    c.tag = (uint8_t)TagOf(packed);
    c.startInside = startInside;
}

// Thickness of the material behind a surface point on the dexel of a leaf: the length of its span
// along the dexel, followed into the next leaves where it goes on, times the component of the
// normal along the dexel: exact for walls between parallel faces
float Thickness(const Grid& grid, int leaf, int a, int i, int j, double s, bool startInside, float na)
{
    const Layout& L = grid.GetLayout();
    const int u = AxisU(a), v = AxisV(a);
    // the material is before s if the edge starts inside
    const double dir = startInside ? -1 : 1;
    double length = 0;
    double at = s;
    int line = LineIndex(a, i, j);
    // world position of the dexel across the axis
    const Layout::Leaf& lf0 = L.GetLeaf(leaf);
    const int pu = lf0.lo[u] + i * lf0.cell, pv = lf0.lo[v] + j * lf0.cell;
    for (int hop = 0; hop < 64; hop++) {
        Span scratch;
        const Span* spans;
        const int n = grid.Line(leaf, line, spans, scratch);
        double rangeLo, rangeHi;
        grid.LineRange(leaf, a, rangeLo, rangeHi);
        // the span holding the material next to 'at'
        int found = -1;
        for (int k = 0; k < n; k++) {
            if (spans[k].lo <= at + 1e-5 && spans[k].hi >= at - 1e-5) {
                found = k;
                break;
            }
        }
        if (found < 0) {
            break;
        }
        const double end = dir < 0 ? spans[found].lo : spans[found].hi;
        length += std::abs(end - at);
        if (length >= MaxThickness) {
            return NoThickness;
        }
        // the span ends inside the dexel: the other face of the material
        const Layout::Leaf& lf = L.GetLeaf(leaf);
        const double cell = lf.cell * L.Unit();
        const bool continues = dir < 0 ? end <= rangeLo + 1e-5 : end >= rangeHi - 1e-5;
        if (!continues) {
            return (float)(length * std::max(0.05f, std::abs(na)));
        }
        // on into the next leaf along the dexel, from the border of this one
        const double border = dir < 0 ? L.Coord(a, lf.lo[a]) : L.Coord(a, lf.lo[a] + Cells * lf.cell);
        length -= std::abs(end - border);
        at = border;
        int p[3];
        p[u] = pu;
        p[v] = pv;
        p[a] = dir < 0 ? lf.lo[a] - 1 : lf.lo[a] + Cells * lf.cell;
        const int next = L.FindLeaf(p[0], p[1], p[2]);
        if (next < 0 || grid.IsEmpty(next)) {
            return (float)(length * std::max(0.05f, std::abs(na)));
        }
        leaf = next;
        const Layout::Leaf& nf = L.GetLeaf(next);
        // the nearest dexel of the next leaf
        const int ni = std::clamp((int)std::lround((double)(pu - nf.lo[u]) / nf.cell), 0, Cells);
        const int nj = std::clamp((int)std::lround((double)(pv - nf.lo[v]) / nf.cell), 0, Cells);
        line = LineIndex(a, ni, nj);
        (void)cell;
    }
    return NoThickness;
}

// Minimizer of the squared distances to the planes of the points, near their mass point: the
// eigenvectors of small eigenvalues are left out, so planes in few directions give the point
// closest to the mass point on their intersection (sharp edges and corners)
void SolveQef(const std::vector<const Crossing*>& points, const double lo[3], const double hi[3], float out[3])
{
    double mass[3] = {0, 0, 0};
    for (const Crossing* c : points) {
        for (int k = 0; k < 3; k++) {
            mass[k] += c->p[k];
        }
    }
    for (int k = 0; k < 3; k++) {
        mass[k] /= (double)points.size();
    }
    double A[3][3] = {}, b[3] = {};
    int withNormal = 0;
    for (const Crossing* c : points) {
        if (!c->hasNormal) {
            continue;
        }
        withNormal++;
        const double d = c->n[0] * (c->p[0] - mass[0]) + c->n[1] * (c->p[1] - mass[1])
            + c->n[2] * (c->p[2] - mass[2]);
        for (int r = 0; r < 3; r++) {
            for (int k = 0; k < 3; k++) {
                A[r][k] += (double)c->n[r] * c->n[k];
            }
            b[r] += c->n[r] * d;
        }
    }
    double x[3] = {0, 0, 0};
    if (withNormal > 0) {
        // Jacobi eigen decomposition of A
        double V[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        double M[3][3];
        for (int r = 0; r < 3; r++) {
            for (int k = 0; k < 3; k++) {
                M[r][k] = A[r][k];
            }
        }
        for (int sweep = 0; sweep < 12; sweep++) {
            const double off = M[0][1] * M[0][1] + M[0][2] * M[0][2] + M[1][2] * M[1][2];
            if (off < 1e-24) {
                break;
            }
            for (int p = 0; p < 2; p++) {
                for (int q = p + 1; q < 3; q++) {
                    if (std::abs(M[p][q]) < 1e-30) {
                        continue;
                    }
                    const double theta = (M[q][q] - M[p][p]) / (2 * M[p][q]);
                    const double t = (theta >= 0 ? 1 : -1) / (std::abs(theta) + std::sqrt(theta * theta + 1));
                    const double c = 1 / std::sqrt(t * t + 1), s = t * c;
                    for (int k = 0; k < 3; k++) {
                        const double mkp = M[k][p], mkq = M[k][q];
                        M[k][p] = c * mkp - s * mkq;
                        M[k][q] = s * mkp + c * mkq;
                    }
                    for (int k = 0; k < 3; k++) {
                        const double mpk = M[p][k], mqk = M[q][k];
                        M[p][k] = c * mpk - s * mqk;
                        M[q][k] = s * mpk + c * mqk;
                    }
                    for (int k = 0; k < 3; k++) {
                        const double vkp = V[k][p], vkq = V[k][q];
                        V[k][p] = c * vkp - s * vkq;
                        V[k][q] = s * vkp + c * vkq;
                    }
                }
            }
        }
        const double maxEig = std::max({M[0][0], M[1][1], M[2][2]});
        for (int e = 0; e < 3; e++) {
            const double lambda = M[e][e];
            if (lambda < 0.1 * maxEig || lambda <= 0) {
                continue;
            }
            double proj = 0;
            for (int k = 0; k < 3; k++) {
                proj += V[k][e] * b[k];
            }
            for (int k = 0; k < 3; k++) {
                x[k] += V[k][e] * proj / lambda;
            }
        }
    }
    for (int k = 0; k < 3; k++) {
        out[k] = (float)std::clamp(mass[k] + x[k], lo[k], hi[k]);
    }
}

}  // namespace

// The cubes of a leaf: cells merged where the surface is flat, with their vertices
struct Mesher::Geometry
{
    struct Cube
    {
        uint8_t lo[3];  // first cell
        uint8_t size;   // cells
        float p[3];     // vertex
        float n[3];     // normal, if hasNormal
        bool hasNormal;
        bool sharp;
        bool cut;
        uint8_t tag;
    };
    std::vector<Cube> cubes;
    uint16_t cubeOf[CellCount];
    // crossing of each edge, -1 for none
    int16_t crossingOf[EdgeCount];
    std::vector<Crossing> crossings;
};

Mesher::Mesher() = default;
Mesher::~Mesher() = default;

void Mesher::Reset(std::shared_ptr<const Layout> layout, ThreadPool& pool)
{
    mLayout = std::move(layout);
    const Layout& L = *mLayout;
    const int leaves = L.LeafCount();
    mGeometry.clear();
    mGeometry.resize(leaves);
    mMeshes.assign(L.UnitCount(), UnitMesh());
    mAll = true;

    // the leaves touching each leaf: at points just outside of it, a quarter of its size apart
    std::vector<std::vector<int>> found(leaves);
    ParallelFor(pool, leaves, 64, [&](int leaf) {
        const Layout::Leaf& lf = L.GetLeaf(leaf);
        const int size = lf.cell * Cells;
        const int offsets[4] = {-1, size / 4, 3 * size / 4, size};
        std::vector<int>& out = found[leaf];
        for (int z = 0; z < 4; z++) {
            for (int y = 0; y < 4; y++) {
                for (int x = 0; x < 4; x++) {
                    if (x > 0 && x < 3 && y > 0 && y < 3 && z > 0 && z < 3) {
                        continue;
                    }
                    const int n = L.FindLeaf(lf.lo[0] + offsets[x], lf.lo[1] + offsets[y], lf.lo[2] + offsets[z]);
                    if (n >= 0 && n != leaf && std::find(out.begin(), out.end(), n) == out.end()) {
                        out.push_back(n);
                    }
                }
            }
        }
    });
    mNeighborStart.assign(leaves + 1, 0);
    mNeighbors.clear();
    for (int leaf = 0; leaf < leaves; leaf++) {
        mNeighbors.insert(mNeighbors.end(), found[leaf].begin(), found[leaf].end());
        mNeighborStart[leaf + 1] = (int)mNeighbors.size();
    }
}

void Mesher::SetMeasureThickness(bool b)
{
    if (b != mThickness) {
        mThickness = b;
        mAll = true;
    }
}

void Mesher::InvalidateAll()
{
    mAll = true;
}

void Mesher::ComputeGeometry(const Grid& grid, int leaf)
{
    if (grid.IsEmpty(leaf) || grid.IsSolid(leaf)) {
        mGeometry[leaf].reset();
        return;
    }
    const Layout& L = *mLayout;
    const Layout::Leaf& lf = L.GetLeaf(leaf);
    auto g = std::make_unique<Geometry>();
    const double cellWorld = lf.cell * L.Unit();

    // inside at the grid points
    bool inside[Cells + 1][Cells + 1][Cells + 1];
    bool any = false, all = true;
    for (int k = 0; k <= Cells; k++) {
        for (int j = 0; j <= Cells; j++) {
            for (int i = 0; i <= Cells; i++) {
                inside[k][j][i] = Inside(grid, leaf, i, j, k);
                any = any || inside[k][j][i];
                all = all && inside[k][j][i];
            }
        }
    }
    // the crossings of the edges
    std::fill(std::begin(g->crossingOf), std::end(g->crossingOf), (int16_t)-1);
    if (any && !all) {
        for (int a = 0; a < 3; a++) {
            const int u = AxisU(a), v = AxisV(a);
            for (int j = 0; j <= Cells; j++) {
                for (int i = 0; i <= Cells; i++) {
                    for (int t = 0; t < Cells; t++) {
                        int p0[3], p1[3];
                        p0[a] = t;
                        p1[a] = t + 1;
                        p0[u] = p1[u] = i;
                        p0[v] = p1[v] = j;
                        const bool s0 = inside[p0[2]][p0[1]][p0[0]];
                        const bool s1 = inside[p1[2]][p1[1]][p1[0]];
                        if (s0 == s1) {
                            continue;
                        }
                        Crossing c;
                        FindCrossing(grid, leaf, a, i, j, t, s0, c);
                        g->crossingOf[EdgeIndex(a, i, j, t)] = (int16_t)g->crossings.size();
                        g->crossings.push_back(c);
                    }
                }
            }
        }
    }

    // crossings of a block of cells: the edges of its cells
    auto gather = [&](int x0, int y0, int z0, int m, std::vector<const Crossing*>& out) {
        const int lo[3] = {x0, y0, z0};
        for (int a = 0; a < 3; a++) {
            const int u = AxisU(a), v = AxisV(a);
            for (int j = lo[v]; j <= lo[v] + m; j++) {
                for (int i = lo[u]; i <= lo[u] + m; i++) {
                    for (int t = lo[a]; t < lo[a] + m; t++) {
                        const int c = g->crossingOf[EdgeIndex(a, i, j, t)];
                        if (c >= 0) {
                            out.push_back(&g->crossings[c]);
                        }
                    }
                }
            }
        }
    };
    // flat: the normals agree and the points are close to a plane
    auto flat = [&](const std::vector<const Crossing*>& points) {
        if (points.empty()) {
            return true;
        }
        double n[3] = {0, 0, 0}, d = 0;
        for (const Crossing* c : points) {
            if (!c->hasNormal) {
                return false;
            }
            for (int k = 0; k < 3; k++) {
                n[k] += c->n[k];
            }
        }
        const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (len <= 0) {
            return false;
        }
        for (double& c : n) {
            c /= len;
        }
        double lo = 1e30, hi = -1e30;
        for (const Crossing* c : points) {
            if (c->n[0] * n[0] + c->n[1] * n[1] + c->n[2] * n[2] < MergeCos) {
                return false;
            }
            d = c->p[0] * n[0] + c->p[1] * n[1] + c->p[2] * n[2];
            lo = std::min(lo, d);
            hi = std::max(hi, d);
        }
        return hi - lo <= 0.2 * cellWorld;
    };

    // merge bottom up: blocks of 2, 4 and 8 cells, if their parts merged and they are flat
    bool merged[4][Cells][Cells][Cells] = {};
    for (int z = 0; z < Cells; z++) {
        for (int y = 0; y < Cells; y++) {
            for (int x = 0; x < Cells; x++) {
                merged[0][z][y][x] = true;
            }
        }
    }
    std::vector<const Crossing*> points;
    for (int level = 1; level <= 3; level++) {
        const int m = 1 << level, count = Cells / m;
        for (int z = 0; z < count; z++) {
            for (int y = 0; y < count; y++) {
                for (int x = 0; x < count; x++) {
                    bool ok = true;
                    for (int c = 0; c < 8 && ok; c++) {
                        ok = merged[level - 1][2 * z + (c >> 2 & 1)][2 * y + (c >> 1 & 1)][2 * x + (c & 1)];
                    }
                    if (ok) {
                        points.clear();
                        gather(x * m, y * m, z * m, m, points);
                        ok = flat(points);
                    }
                    merged[level][z][y][x] = ok;
                }
            }
        }
    }
    // the cubes, from the largest merged blocks
    for (int c = 0; c < CellCount; c++) {
        g->cubeOf[c] = 0xFFFF;
    }
    for (int level = 3; level >= 0; level--) {
        const int m = 1 << level, count = Cells / m;
        for (int z = 0; z < count; z++) {
            for (int y = 0; y < count; y++) {
                for (int x = 0; x < count; x++) {
                    if (!merged[level][z][y][x] || g->cubeOf[CellIndex(x * m, y * m, z * m)] != 0xFFFF) {
                        continue;
                    }
                    const uint16_t index = (uint16_t)g->cubes.size();
                    Geometry::Cube cube {};
                    cube.lo[0] = (uint8_t)(x * m);
                    cube.lo[1] = (uint8_t)(y * m);
                    cube.lo[2] = (uint8_t)(z * m);
                    cube.size = (uint8_t)m;
                    g->cubes.push_back(cube);
                    for (int cz = z * m; cz < z * m + m; cz++) {
                        for (int cy = y * m; cy < y * m + m; cy++) {
                            for (int cx = x * m; cx < x * m + m; cx++) {
                                g->cubeOf[CellIndex(cx, cy, cz)] = index;
                            }
                        }
                    }
                }
            }
        }
    }

    // surface points of finer neighbors on the faces of the leaf, by cube
    std::vector<std::vector<Crossing>> extra(g->cubes.size());
    for (int ni = mNeighborStart[leaf]; ni < mNeighborStart[leaf + 1]; ni++) {
        const int other = mNeighbors[ni];
        const Layout::Leaf& of = L.GetLeaf(other);
        if (of.level <= lf.level || grid.IsEmpty(other) || grid.IsSolid(other)) {
            continue;
        }
        // the face shared with the finer leaf, if any
        const int size = lf.cell * Cells, osize = of.cell * Cells;
        for (int a = 0; a < 3; a++) {
            int plane;
            if (of.lo[a] + osize == lf.lo[a]) {
                plane = Cells;  // its upper face is our lower face
            }
            else if (of.lo[a] == lf.lo[a] + size) {
                plane = 0;
            }
            else {
                continue;
            }
            const int u = AxisU(a), v = AxisV(a);
            if (of.lo[u] < lf.lo[u] || of.lo[u] >= lf.lo[u] + size || of.lo[v] < lf.lo[v]
                || of.lo[v] >= lf.lo[v] + size) {
                continue;
            }
            // edges of the finer leaf on the plane, along u and v
            for (int b : {u, v}) {
                const int bu = AxisU(b), bv = AxisV(b);
                for (int j = 0; j <= Cells; j++) {
                    for (int i = 0; i <= Cells; i++) {
                        if ((bu == a ? i : j) != plane) {
                            continue;
                        }
                        for (int t = 0; t < Cells; t++) {
                            int p0[3], p1[3];
                            p0[b] = t;
                            p1[b] = t + 1;
                            p0[bu] = p1[bu] = i;
                            p0[bv] = p1[bv] = j;
                            const bool s0 = Inside(grid, other, p0[0], p0[1], p0[2]);
                            const bool s1 = Inside(grid, other, p1[0], p1[1], p1[2]);
                            if (s0 == s1) {
                                continue;
                            }
                            Crossing c;
                            FindCrossing(grid, other, b, i, j, t, s0, c);
                            // the cell of this leaf at the point
                            int cell[3];
                            for (int k = 0; k < 3; k++) {
                                const double units = (c.p[k] - L.Origin()[k]) / L.Unit() - lf.lo[k];
                                cell[k] = std::clamp((int)std::floor(units / lf.cell), 0, Cells - 1);
                            }
                            extra[g->cubeOf[CellIndex(cell[0], cell[1], cell[2])]].push_back(c);
                        }
                    }
                }
            }
        }
    }

    // the vertices
    for (size_t ci = 0; ci < g->cubes.size(); ci++) {
        Geometry::Cube& cube = g->cubes[ci];
        points.clear();
        gather(cube.lo[0], cube.lo[1], cube.lo[2], cube.size, points);
        for (const Crossing& c : extra[ci]) {
            points.push_back(&c);
        }
        double lo[3], hi[3];
        for (int k = 0; k < 3; k++) {
            lo[k] = L.Coord(k, lf.lo[k] + cube.lo[k] * lf.cell);
            hi[k] = L.Coord(k, lf.lo[k] + (cube.lo[k] + cube.size) * lf.cell);
        }
        if (points.empty()) {
            for (int k = 0; k < 3; k++) {
                cube.p[k] = (float)(0.5 * (lo[k] + hi[k]));
            }
            cube.hasNormal = false;
            continue;
        }
        SolveQef(points, lo, hi, cube.p);
        double n[3] = {0, 0, 0};
        int withNormal = 0, tagVotes[64] = {};
        int cut = 0;
        for (const Crossing* c : points) {
            cut += c->cut ? 1 : 0;
            tagVotes[c->tag & 63]++;
            if (c->hasNormal) {
                withNormal++;
                for (int k = 0; k < 3; k++) {
                    n[k] += c->n[k];
                }
            }
        }
        cube.cut = 2 * cut >= (int)points.size();
        cube.tag = (uint8_t)(std::max_element(tagVotes, tagVotes + 64) - tagVotes);
        const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        cube.hasNormal = withNormal > 0 && len > 1e-6;
        cube.sharp = withNormal < (int)points.size();
        if (cube.hasNormal) {
            for (int k = 0; k < 3; k++) {
                cube.n[k] = (float)(n[k] / len);
            }
            for (const Crossing* c : points) {
                if (c->hasNormal
                    && c->n[0] * cube.n[0] + c->n[1] * cube.n[1] + c->n[2] * cube.n[2] < SharpCos) {
                    cube.sharp = true;
                }
            }
        }
    }
    mGeometry[leaf] = std::move(g);
}

void Mesher::BuildUnit(const Grid& grid, int unit, UnitMesh& out) const
{
    out.vertices.clear();
    out.stock.clear();
    out.cut.clear();
    const Layout& L = *mLayout;
    // shared vertices by leaf and cube
    std::unordered_map<uint64_t, uint32_t> shared;
    float thickness = NoThickness;  // of the quad being made
    auto vertexOf = [&](int leaf, int cube, const Crossing& c) -> uint32_t {
        const Geometry::Cube& q = mGeometry[leaf]->cubes[cube];
        const bool smooth = q.hasNormal && !q.sharp;
        const uint64_t key = ((uint64_t)leaf << 16) | (uint64_t)cube;
        if (smooth) {
            auto it = shared.find(key);
            if (it != shared.end()) {
                float& t = out.vertices[it->second].thickness;
                t = std::min(t, thickness);
                return it->second;
            }
        }
        MeshVertex v;
        v.x = q.p[0];
        v.y = q.p[1];
        v.z = q.p[2];
        const float* n = smooth ? q.n : c.n;
        v.nx = (int8_t)std::lround(std::clamp(n[0], -1.0f, 1.0f) * 127);
        v.ny = (int8_t)std::lround(std::clamp(n[1], -1.0f, 1.0f) * 127);
        v.nz = (int8_t)std::lround(std::clamp(n[2], -1.0f, 1.0f) * 127);
        v.tag = smooth ? q.tag : c.tag;
        v.thickness = thickness;
        const uint32_t index = (uint32_t)out.vertices.size();
        out.vertices.push_back(v);
        if (smooth) {
            shared.emplace(key, index);
        }
        return index;
    };

    for (int leaf = L.UnitBegin(unit); leaf < L.UnitEnd(unit); leaf++) {
        const Geometry* g = mGeometry[leaf].get();
        if (!g || g->crossings.empty()) {
            continue;
        }
        const Layout::Leaf& lf = L.GetLeaf(leaf);
        Span scratch;
        (void)scratch;
        for (int ci = 0; ci < (int)g->cubes.size(); ci++) {
            const Geometry::Cube& q = g->cubes[ci];
            const int m = q.size;
            for (int a = 0; a < 3; a++) {
                const int u = AxisU(a), v = AxisV(a);
                for (int dv = 0; dv <= 1; dv++) {
                    for (int du = 0; du <= 1; du++) {
                        // the edge from grid point s along a, m cells long
                        int s[3];
                        s[a] = q.lo[a];
                        s[u] = q.lo[u] + du * m;
                        s[v] = q.lo[v] + dv * m;
                        // the fine edge with the change of inside, if the ends differ
                        int crossing = -1, changes = 0;
                        for (int t = s[a]; t < s[a] + m; t++) {
                            const int c = g->crossingOf[EdgeIndex(a, s[u], s[v], t)];
                            if (c >= 0) {
                                changes++;
                                if (crossing < 0) {
                                    crossing = c;
                                }
                            }
                        }
                        if ((changes & 1) == 0) {
                            continue;
                        }
                        const Crossing& cr = g->crossings[crossing];
                        // the cubes in the four quadrants around the edge
                        int leaves[4], cubes[4], sizes[4];
                        bool missing = false;
                        for (int quad = 0; quad < 4; quad++) {
                            const int qu = quad & 1, qv = quad >> 1;  // 0: below, 1: above
                            int cell[3];
                            cell[a] = s[a];
                            cell[u] = s[u] - (qu ? 0 : 1);
                            cell[v] = s[v] - (qv ? 0 : 1);
                            int nl = leaf, nc[3] = {cell[0], cell[1], cell[2]};
                            if (cell[u] < 0 || cell[u] >= Cells || cell[v] < 0 || cell[v] >= Cells) {
                                const int px = lf.lo[0] + cell[0] * lf.cell;
                                const int py = lf.lo[1] + cell[1] * lf.cell;
                                const int pz = lf.lo[2] + cell[2] * lf.cell;
                                nl = L.FindLeaf(px, py, pz);
                                if (nl < 0 || !mGeometry[nl]) {
                                    missing = true;
                                    break;
                                }
                                const Layout::Leaf& nf = L.GetLeaf(nl);
                                nc[0] = (px - nf.lo[0]) / nf.cell;
                                nc[1] = (py - nf.lo[1]) / nf.cell;
                                nc[2] = (pz - nf.lo[2]) / nf.cell;
                            }
                            const Geometry* ng = mGeometry[nl].get();
                            leaves[quad] = nl;
                            cubes[quad] = ng->cubeOf[CellIndex(nc[0], nc[1], nc[2])];
                            sizes[quad] = ng->cubes[cubes[quad]].size * L.GetLeaf(nl).cell;
                        }
                        if (missing) {
                            continue;
                        }
                        // from the smallest cubes, once: by the first of the smallest
                        const int mySize = m * lf.cell;
                        bool smaller = false;
                        int owner = -1;
                        for (int quad = 0; quad < 4; quad++) {
                            smaller = smaller || sizes[quad] < mySize;
                            if (owner < 0 && sizes[quad] == mySize) {
                                owner = quad;
                            }
                        }
                        if (smaller || leaves[owner] != leaf || cubes[owner] != ci) {
                            continue;
                        }
                        thickness = mThickness
                            ? Thickness(grid, leaf, a, s[u], s[v], cr.p[a], cr.startInside, cr.n[a])
                            : NoThickness;
                        // the vertices around the edge, counter clockwise seen along +a
                        const int order[4] = {0, 1, 3, 2};
                        uint32_t idx[4];
                        for (int k = 0; k < 4; k++) {
                            idx[k] = vertexOf(leaves[order[k]], cubes[order[k]], cr);
                        }
                        // the surface faces +a if the edge starts inside
                        const bool facesUp = cr.startInside;
                        if (!facesUp) {
                            std::swap(idx[1], idx[3]);
                        }
                        std::vector<uint32_t>& list = cr.cut ? out.cut : out.stock;
                        // distinct vertices: two triangles, or one
                        uint32_t uniq[4];
                        int nu = 0;
                        for (int k = 0; k < 4; k++) {
                            bool seen = false;
                            for (int l = 0; l < nu; l++) {
                                const MeshVertex& a = out.vertices[uniq[l]];
                                const MeshVertex& b = out.vertices[idx[k]];
                                seen = seen || (a.x == b.x && a.y == b.y && a.z == b.z);
                            }
                            if (!seen) {
                                uniq[nu++] = idx[k];
                            }
                        }
                        if (nu == 4) {
                            list.insert(list.end(), {idx[0], idx[1], idx[2], idx[0], idx[2], idx[3]});
                        }
                        else if (nu == 3) {
                            list.insert(list.end(), {uniq[0], uniq[1], uniq[2]});
                        }
                    }
                }
            }
        }
    }
}

std::vector<int> Mesher::Update(const Grid& grid, const std::vector<int>& changed, ThreadPool& pool)
{
    const Layout& L = *mLayout;
    const int leaves = L.LeafCount();
    std::vector<int> geometry, units;
    std::vector<uint8_t> mark(leaves, 0);
    if (mAll) {
        mAll = false;
        geometry.resize(leaves);
        for (int i = 0; i < leaves; i++) {
            geometry[i] = i;
        }
        units.resize(L.UnitCount());
        for (int i = 0; i < L.UnitCount(); i++) {
            units[i] = i;
        }
    }
    else {
        // geometry of the changed leaves and of their neighbors (points on shared faces)
        for (int leaf : changed) {
            if (!mark[leaf]) {
                mark[leaf] = 1;
                geometry.push_back(leaf);
            }
            for (int n = mNeighborStart[leaf]; n < mNeighborStart[leaf + 1]; n++) {
                if (!mark[mNeighbors[n]]) {
                    mark[mNeighbors[n]] = 1;
                    geometry.push_back(mNeighbors[n]);
                }
            }
        }
        // units using those cubes: of the leaves and their neighbors
        std::vector<uint8_t> unitMark(L.UnitCount(), 0);
        for (int leaf : geometry) {
            for (int n = mNeighborStart[leaf] - 1; n < mNeighborStart[leaf + 1]; n++) {
                const int l = n < mNeighborStart[leaf] ? leaf : mNeighbors[n];
                const int u = L.UnitOf(l);
                if (!unitMark[u]) {
                    unitMark[u] = 1;
                    units.push_back(u);
                }
            }
        }
    }
    ParallelFor(pool, (int)geometry.size(), 16, [&](int i) { ComputeGeometry(grid, geometry[i]); });
    ParallelFor(pool, (int)units.size(), 4, [&](int i) { BuildUnit(grid, units[i], mMeshes[units[i]]); });
    return units;
}

}  // namespace CAMSimulator::Material
