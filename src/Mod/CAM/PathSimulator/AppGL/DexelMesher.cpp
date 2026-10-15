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


#include "DexelMesher.h"
#include "ThreadPool.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <tuple>
#include <unordered_map>

namespace CAMSimulator
{

namespace
{

constexpr int GridPoints = BrickLines * BrickLines * BrickLines;
constexpr int Cells = BrickCells * BrickCells * BrickCells;
constexpr int EdgesPerAxis = BrickAxisLines * BrickCells;
// log2 of BrickCells
constexpr int BrickLog = 3;
// Crossings with normals closer than this (cosine) share a vertex normal: smooth shading
constexpr float ClusterCos = 0.7f;
constexpr int MaxClusters = 4;
// Directions of the QEF with less than this fraction of the largest weight are not used, so
// vertices of smooth surfaces stay near the average crossing, and only sharper edges are
// reproduced
constexpr double QefTolerance = 0.08;
// Cells of a brick are merged into larger cubes where one vertex fits the planes of their
// crossings within this fraction of a cell, so flat and gently curved surfaces get fewer and
// larger polygons
constexpr double MergeTolerance = 0.1;
// ... and where the crossing normals of each cluster deviate from their mean by less than this
// (1 - cosine, about 8 degrees), so the shading of shallow features like the scallops of a ball
// end mill is kept
constexpr double MergeNormalTolerance = 0.01;
// A ridge of a cut surface, like between the passes of a ball end mill, where the surface bends
// convex between neighboring lines by more than this (about 6 degrees) and more than this many
// times as much as on either side. A smooth convex surface bends about the same everywhere.
constexpr double RidgeBend = 0.1;
constexpr double RidgeRatio = 2;
// Ridges through a cell at most
constexpr int MaxRidges = 2;

inline int AxisU(int a)
{
    return (a + 1) % 3;
}
inline int AxisV(int a)
{
    return (a + 2) % 3;
}
inline int PointIndex(const int p[3])
{
    return p[0] + BrickLines * (p[1] + BrickLines * p[2]);
}
inline int CellIndex(const int c[3])
{
    return c[0] + BrickCells * (c[1] + BrickCells * c[2]);
}
inline void CellCoords(int cell, int c[3])
{
    c[0] = cell % BrickCells;
    c[1] = cell / BrickCells % BrickCells;
    c[2] = cell / (BrickCells * BrickCells);
}
// Grid edge along axis a, on the line (i, j), from point k to k + 1
inline int EdgeIndex(int a, int i, int j, int k)
{
    return a * EdgesPerAxis + (j * BrickLines + i) * BrickCells + k;
}
// Distance between grid points along an axis, in point indices
constexpr int PointStride[3] = {1, BrickLines, BrickLines * BrickLines};

}  // namespace

struct LeafGeometry
{
    struct Crossing
    {
        float t;
        uint32_t normal;
        bool valid;
        float thickness = ThicknessUnknown;  // of the material behind it, if measured
    };
    // Crossing normals of a cube close to each other, with the number of cut and stock faces
    struct Cluster
    {
        float n[3];
        int cut;
        int stock;
        uint8_t side;       // the side of the ridges through the cube
        uint8_t operation;  // tag of the operation that cut the faces
        float thickness;    // smallest thickness of the material behind the crossings
    };
    struct Vertex
    {
        float p[3];
        uint16_t cell;  // first cell of its cube
        uint16_t firstCluster;
        uint8_t clusters;
    };

    uint64_t inside[(GridPoints + 63) / 64] = {};
    // cubes of merged cells: 2^3 (bit gx + 4 (gy + 4 gz)), 4^3 (bit gx + 2 (gy + 2 gz)), 8^3
    uint64_t merged2 = 0;
    uint8_t merged4 = 0;
    bool merged8 = false;
    std::vector<uint16_t> edges;  // edges with a change of inside, sorted
    std::vector<Crossing> crossings;
    std::vector<Vertex> vertices;  // sorted by cell
    std::vector<Cluster> clusters;

    bool Inside(int point) const
    {
        return ((inside[point >> 6] >> (point & 63)) & 1) != 0;
    }
    int CubeLog(const int c[3]) const
    {
        if (merged8) {
            return 3;
        }
        if ((merged4 >> ((c[0] >> 2) + 2 * ((c[1] >> 2) + 2 * (c[2] >> 2)))) & 1) {
            return 2;
        }
        if ((merged2 >> ((c[0] >> 1) + 4 * ((c[1] >> 1) + 4 * (c[2] >> 1)))) & 1) {
            return 1;
        }
        return 0;
    }
    const Vertex* VertexAt(int cell) const
    {
        const auto it = std::lower_bound(vertices.begin(), vertices.end(), cell, [](const Vertex& v, int c) {
            return v.cell < c;
        });
        return it != vertices.end() && it->cell == cell ? &*it : nullptr;
    }
    const Crossing* CrossingAt(int edge) const
    {
        const auto it = std::lower_bound(edges.begin(), edges.end(), (uint16_t)edge);
        return it != edges.end() && *it == edge ? &crossings[it - edges.begin()] : nullptr;
    }

    // crossings of the edges of merged cubes with a change, by CoarseKey
    std::vector<std::pair<uint32_t, Crossing>> coarse;
    static uint32_t CoarseKey(int a, int i, int j, int k, int log)
    {
        return (((a * BrickLines + j) * BrickLines + i) * BrickCells + k) * 4 + log;
    }
    Crossing CoarseCrossingAt(uint32_t key) const
    {
        const auto it = std::lower_bound(coarse.begin(), coarse.end(), key, [](const auto& c, uint32_t k) {
            return c.first < k;
        });
        return it != coarse.end() && it->first == key ? it->second : Crossing {0, 0, false};
    }
};

namespace
{

using Crossing = LeafGeometry::Crossing;
using Cluster = LeafGeometry::Cluster;

// Planes of the crossings of a cube, positions relative to the center of its brick
struct Qef
{
    double ata[3][3];
    double atb[3];
    double btb;
    double mass[3];
    int count;   // planes
    int points;  // crossings, with or without normal
    int nClusters;
    Cluster clusters[MaxClusters];
    bool incomplete;  // edges with a change but no crossing, or crossings without normal
    bool ridge;       // a ridge goes through the cube, its sides have separate normals

    void Clear()
    {
        *this = Qef {};
    }

    void AddCluster(const float n[3], int cut, int stock, uint8_t side, uint8_t operation, float thickness)
    {
        int best = -1;
        float bestDot = ClusterCos;
        const float nlen = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        for (int i = 0; i < nClusters; i++) {
            if (clusters[i].side != side || clusters[i].operation != operation) {
                continue;
            }
            const float* s = clusters[i].n;
            const float len = std::sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
            const float dot = (s[0] * n[0] + s[1] * n[1] + s[2] * n[2]) / (len * nlen);
            if (dot > bestDot) {
                bestDot = dot;
                best = i;
            }
        }
        if (best < 0 && nClusters < MaxClusters) {
            best = nClusters++;
            clusters[best] = {{0, 0, 0}, 0, 0, side, operation, ThicknessUnknown};
        }
        if (best < 0) {
            best = 0;
        }
        for (int k = 0; k < 3; k++) {
            clusters[best].n[k] += n[k];
        }
        clusters[best].cut += cut;
        clusters[best].stock += stock;
        clusters[best].thickness = std::min(clusters[best].thickness, thickness);
    }

    void Add(const double p[3], const float n[3], bool cut, uint8_t side, uint8_t operation, float thickness)
    {
        const double d = n[0] * p[0] + n[1] * p[1] + n[2] * p[2];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                ata[r][c] += (double)n[r] * n[c];
            }
            atb[r] += n[r] * d;
            mass[r] += p[r];
        }
        btb += d * d;
        count++;
        points++;
        AddCluster(n, cut ? 1 : 0, cut ? 0 : 1, side, operation, thickness);
    }

    // A crossing without normal: its position only
    void AddPoint(const double p[3])
    {
        for (int r = 0; r < 3; r++) {
            mass[r] += p[r];
        }
        points++;
        incomplete = true;
    }

    void Add(const double p[3], const Crossing& x, uint8_t side = 0)
    {
        if (!HasNormal(x.normal)) {
            AddPoint(p);
            return;
        }
        float n[3];
        UnpackNormal(x.normal, n);
        Add(p, n, IsCutFace(x.normal), side, (uint8_t)OperationTagOf(x.normal), x.thickness);
    }

    void Merge(const Qef& o)
    {
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                ata[r][c] += o.ata[r][c];
            }
            atb[r] += o.atb[r];
            mass[r] += o.mass[r];
        }
        btb += o.btb;
        count += o.count;
        points += o.points;
        incomplete |= o.incomplete;
        ridge |= o.ridge;
        for (int i = 0; i < o.nClusters; i++) {
            const Cluster& c = o.clusters[i];
            AddCluster(c.n, c.cut, c.stock, c.side, c.operation, c.thickness);
        }
    }

    // Largest mean deviation of the normals of a cluster from their mean, 1 - cosine
    double NormalSpread() const
    {
        double spread = 0;
        for (int i = 0; i < nClusters; i++) {
            const float* n = clusters[i].n;
            const int k = clusters[i].cut + clusters[i].stock;
            if (k > 1) {
                const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
                spread = std::max(spread, 1 - len / k);
            }
        }
        return spread;
    }

    bool Mixed() const
    {
        int cut = 0, stock = 0;
        for (int i = 0; i < nClusters; i++) {
            cut += clusters[i].cut;
            stock += clusters[i].stock;
        }
        return cut > 0 && stock > 0;
    }

    // Sum of the squared distances of x from the planes
    double Error(const double x[3]) const
    {
        double e = btb;
        for (int r = 0; r < 3; r++) {
            e -= 2 * atb[r] * x[r];
            for (int c = 0; c < 3; c++) {
                e += x[r] * ata[r][c] * x[c];
            }
        }
        return std::max(0.0, e);
    }

    // Point minimizing the distance to the planes, near the mass point
    void Solve(double out[3]) const
    {
        double m[3];
        for (int k = 0; k < 3; k++) {
            m[k] = mass[k] / points;
        }
        if (count == 0) {
            std::copy(m, m + 3, out);
            return;
        }
        // right hand side relative to the mass point
        double b[3];
        for (int r = 0; r < 3; r++) {
            b[r] = atb[r] - (ata[r][0] * m[0] + ata[r][1] * m[1] + ata[r][2] * m[2]);
        }
        // eigen decomposition of the symmetric matrix, Jacobi rotations
        double a[3][3], v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                a[r][c] = ata[r][c];
            }
        }
        for (int sweep = 0; sweep < 8; sweep++) {
            const double off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
            if (off < 1e-20) {
                break;
            }
            for (int p = 0; p < 2; p++) {
                for (int q = p + 1; q < 3; q++) {
                    if (std::abs(a[p][q]) < 1e-30) {
                        continue;
                    }
                    const double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                    const double t = (theta >= 0 ? 1.0 : -1.0)
                        / (std::abs(theta) + std::sqrt(theta * theta + 1));
                    const double c = 1 / std::sqrt(t * t + 1), s = t * c;
                    for (int k = 0; k < 3; k++) {
                        const double akp = a[k][p], akq = a[k][q];
                        a[k][p] = c * akp - s * akq;
                        a[k][q] = s * akp + c * akq;
                    }
                    for (int k = 0; k < 3; k++) {
                        const double apk = a[p][k], aqk = a[q][k];
                        a[p][k] = c * apk - s * aqk;
                        a[q][k] = s * apk + c * aqk;
                    }
                    for (int k = 0; k < 3; k++) {
                        const double vkp = v[k][p], vkq = v[k][q];
                        v[k][p] = c * vkp - s * vkq;
                        v[k][q] = s * vkp + c * vkq;
                    }
                }
            }
        }
        const double largest = std::max({a[0][0], a[1][1], a[2][2]});
        for (int k = 0; k < 3; k++) {
            out[k] = m[k];
        }
        for (int e = 0; e < 3; e++) {
            const double lambda = a[e][e];
            if (lambda <= QefTolerance * largest || lambda <= 0) {
                continue;
            }
            const double proj = (v[0][e] * b[0] + v[1][e] * b[1] + v[2][e] * b[2]) / lambda;
            for (int k = 0; k < 3; k++) {
                out[k] += proj * v[k][e];
            }
        }
    }
};

// Crossing on the edge from t0 to t1 of a line: where the material containing t0 ends (in0) or
// the next material starts. The inside state of the grid points comes from the Z dexels, other
// dexels can disagree slightly near a surface: then the closest end of material of the same kind
// within an edge length, a point of the surface just beyond the edge. The ends of the line at lo
// and hi, where its material is clipped, are not surfaces.
Crossing FindCrossing(const Span* spans, int n, float t0, float t1, bool in0, float lo, float hi)
{
    for (int s = 0; s < n; s++) {
        if (in0) {
            if (spans[s].hi > t0) {
                if (spans[s].hi <= t1) {
                    return {spans[s].hi, spans[s].nhi, true};
                }
                break;
            }
        }
        else if (spans[s].lo > t0) {
            if (spans[s].lo <= t1) {
                return {spans[s].lo, spans[s].nlo, true};
            }
            break;
        }
    }
    const float reach = t1 - t0;
    float best = reach;
    Crossing found {0, 0, false};
    for (int s = 0; s < n; s++) {
        const float t = in0 ? spans[s].hi : spans[s].lo;
        if (t <= lo || t >= hi) {
            continue;
        }
        const float d = t < t0 ? t0 - t : t > t1 ? t - t1 : 0;
        if (d <= best) {
            best = d;
            found = {t, in0 ? spans[s].nhi : spans[s].nlo, true};
        }
    }
    return found;
}

// The inside state at the middle of every edge, face and the cube of 3 x 3 x 3 points (origin g,
// spacing 'half') equals that of one of their corners: merging the 8 sub-cubes into one keeps
// the shape of the surface
bool SafeToMerge(const uint8_t* in, const int g[3], int half)
{
    auto at = [&](int x, int y, int z) {
        const int p[3] = {g[0] + x * half, g[1] + y * half, g[2] + z * half};
        return in[PointIndex(p)];
    };
    for (int x = 0; x <= 2; x++) {
        for (int y = 0; y <= 2; y++) {
            for (int z = 0; z <= 2; z++) {
                if (x != 1 && y != 1 && z != 1) {
                    continue;  // a corner
                }
                const uint8_t s = at(x, y, z);
                bool match = false;
                for (int cx = 0; cx <= 2 && !match; cx += 2) {
                    for (int cy = 0; cy <= 2 && !match; cy += 2) {
                        for (int cz = 0; cz <= 2 && !match; cz += 2) {
                            if ((x != 1 && cx != x) || (y != 1 && cy != y) || (z != 1 && cz != z)) {
                                continue;
                            }
                            match = at(cx, cy, cz) == s;
                        }
                    }
                }
                if (!match) {
                    return false;
                }
            }
        }
    }
    return true;
}

// Normal of a crossing without one, from the slope of the surface between the crossings of the
// neighboring parallel dexels: near the crossing the surface is a height field along the dexel.
// Left without normal if the surface is too steep for this direction.
void EstimateNormal(const DexelStock& stock, int leaf, int a, int i, int j, bool end, float h, Crossing& x)
{
    // the end of material (end) or start next to t on a neighboring line
    auto neighbor = [&](int ni, int nj, float& out) {
        if (ni < 0 || ni >= BrickLines || nj < 0 || nj >= BrickLines) {
            return false;
        }
        Span scratch;
        const Span* spans;
        const int n = stock.Line(leaf, a * BrickAxisLines + nj * BrickLines + ni, spans, scratch);
        float best = 3 * h;
        bool found = false;
        for (int s = 0; s < n; s++) {
            const float e = end ? spans[s].hi : spans[s].lo;
            if (std::abs(e - x.t) < best) {
                best = std::abs(e - x.t);
                out = e;
                found = true;
            }
        }
        return found;
    };
    float slope[2];
    for (int d = 0; d < 2; d++) {
        float plus = 0, minus = 0;
        const bool hasPlus = d == 0 ? neighbor(i + 1, j, plus) : neighbor(i, j + 1, plus);
        const bool hasMinus = d == 0 ? neighbor(i - 1, j, minus) : neighbor(i, j - 1, minus);
        if (hasPlus && hasMinus) {
            slope[d] = (plus - minus) / (2 * h);
        }
        else if (hasPlus) {
            slope[d] = (plus - x.t) / h;
        }
        else if (hasMinus) {
            slope[d] = (x.t - minus) / h;
        }
        else {
            return;
        }
        if (std::abs(slope[d]) > 2) {
            return;
        }
    }
    // surface t = f(u, v), the material before it along the axis if 'end'
    const float sign = end ? 1.0f : -1.0f;
    float n[3];
    n[AxisU(a)] = -slope[0] * sign;
    n[AxisV(a)] = -slope[1] * sign;
    n[a] = sign;
    x.normal = WithOperationTag(PackNormal(n, IsCutFace(x.normal)), OperationTagOf(x.normal));
}

// Inside state, crossings, merged cubes and vertices of a leaf. Returns nullptr if the leaf has
// no surface, 'solid' tells if it is inside then.
// Length of the material along the line through (pu, pv) along axis a from t, in direction dir
// (1 or -1), followed through the leaves ahead, at most 'cap', and the normal where it ends
// (NoNormal if it doesn't end within the cap)
float MaterialRun(const DexelStock& stock, int a, float pu, float pv, float t, int dir, float cap, uint32_t& endNormal)
{
    const DexelLayout& layout = stock.Layout();
    const int u = AxisU(a), v = AxisV(a);
    const float fine = layout.FineCell();
    endNormal = NoNormal;
    int f[3];
    f[u] = layout.FineIndex(u, pu + 0.25f * fine);
    f[v] = layout.FineIndex(v, pv + 0.25f * fine);
    float pos = t;
    Span scratch;
    // a leaf at most every iteration, the cap stops it long before in practice
    for (int guard = 0; guard < 4096; guard++) {
        const float run = std::abs(pos - t);
        if (run >= cap) {
            return cap;
        }
        f[a] = layout.FineIndex(a, pos + dir * 0.25f * fine);
        const int leaf = layout.FindLeaf(f[0], f[1], f[2]);
        if (leaf < 0 || stock.IsEmpty(leaf)) {
            return run;
        }
        const LayoutLeaf& l = layout.Leaf(leaf);
        const float leafEnd = layout.Coord(a, l.origin[a] + (dir > 0 ? BrickCells * l.step : 0));
        if (stock.IsSolid(leaf)) {
            pos = leafEnd;
            continue;
        }
        // the line of the leaf closest to the line
        const int i = std::clamp((int)std::lround((f[u] - l.origin[u]) / (double)l.step), 0, BrickLines - 1);
        const int j = std::clamp((int)std::lround((f[v] - l.origin[v]) / (double)l.step), 0, BrickLines - 1);
        const Span* spans;
        const int n = stock.Line(leaf, a * BrickAxisLines + j * BrickLines + i, spans, scratch);
        float lo, hi;
        stock.LineRange(leaf, a, lo, hi);
        const float eps = 1e-3f * l.step * fine;
        const Span* span = nullptr;
        for (int s = 0; s < n && !span; s++) {
            if (spans[s].lo - eps <= pos && pos <= spans[s].hi + eps) {
                span = &spans[s];
            }
        }
        if (!span) {
            return run;
        }
        const float end = dir > 0 ? span->hi : span->lo;
        const bool clipped = dir > 0 ? end >= hi - eps : end <= lo + eps;
        if (!clipped) {
            endNormal = dir > 0 ? span->nhi : span->nlo;
            return std::min(cap, std::abs(end - t));
        }
        pos = leafEnd;
    }
    return cap;
}

// Thickness of the material behind a crossing of the line through (pu, pv) along axis a: the
// length of the material along the line times the component of the normals along it, exact for
// walls with flat faces. Lines grazing a convex surface, ending where the surface doesn't face the
// other way, measure nothing.
float CrossingThickness(const DexelStock& stock, int a, float pu, float pv, const Crossing& x, bool in0)
{
    if (!x.valid || !HasNormal(x.normal)) {
        return ThicknessUnknown;
    }
    uint32_t endNormal;
    const float run = MaterialRun(stock, a, pu, pv, x.t, in0 ? -1 : 1, MaxMeasuredThickness, endNormal);
    if (run >= MaxMeasuredThickness) {
        return ThicknessUnknown;
    }
    float n1[3];
    UnpackNormal(x.normal, n1);
    float along = std::abs(n1[a]);
    if (HasNormal(endNormal)) {
        float n2[3];
        UnpackNormal(endNormal, n2);
        if (n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2] > -0.5f) {
            return ThicknessUnknown;
        }
        along = (along + std::abs(n2[a])) / 2;
    }
    return run * along;
}

std::unique_ptr<LeafGeometry> ComputeLeaf(const DexelStock& stock, int leaf, bool& solid, bool measure)
{
    const DexelLayout& layout = stock.Layout();
    const LayoutLeaf& l = layout.Leaf(leaf);
    solid = stock.IsSolid(leaf);
    if (solid || stock.IsEmpty(leaf)) {
        return nullptr;
    }

    // inside from the Z dexels, a span contains [lo, hi)
    uint8_t in[GridPoints];
    Span scratch;
    float z[BrickLines];
    for (int k = 0; k < BrickLines; k++) {
        z[k] = layout.Coord(2, l.origin[2] + k * l.step);
    }
    for (int y = 0; y < BrickLines; y++) {
        for (int x = 0; x < BrickLines; x++) {
            const Span* spans;
            const int n = stock.Line(leaf, 2 * BrickAxisLines + y * BrickLines + x, spans, scratch);
            int s = 0;
            for (int k = 0; k < BrickLines; k++) {
                while (s < n && spans[s].hi <= z[k]) {
                    s++;
                }
                in[x + BrickLines * (y + BrickLines * k)] = s < n && spans[s].lo <= z[k];
            }
        }
    }
    bool uniform = true;
    for (int i = 1; i < GridPoints && uniform; i++) {
        uniform = in[i] == in[0];
    }
    if (uniform) {
        solid = in[0] != 0;
        return nullptr;
    }

    auto g = std::make_unique<LeafGeometry>();
    for (int i = 0; i < GridPoints; i++) {
        if (in[i]) {
            g->inside[i >> 6] |= uint64_t(1) << (i & 63);
        }
    }
    double center[3];
    for (int k = 0; k < 3; k++) {
        center[k] = ((double)layout.Coord(k, l.origin[k]) + layout.Coord(k, l.origin[k] + BrickCells * l.step)) / 2;
    }
    const double h = l.step * (double)layout.FineCell();

    // planes of the cells, from the crossings on the edges with a change
    thread_local std::vector<Qef> qefs;
    qefs.clear();
    int16_t qIndex[Cells];
    std::fill(std::begin(qIndex), std::end(qIndex), (int16_t)-1);
    auto qefOf = [&](int cell) -> Qef& {
        if (qIndex[cell] < 0) {
            qIndex[cell] = (int16_t)qefs.size();
            qefs.emplace_back();
            qefs.back().Clear();
        }
        return qefs[qIndex[cell]];
    };
    // crossings of the edges with a change, positions relative to the center
    struct EdgeCrossing
    {
        int a, i, j, k;
        bool in0;
        double p[3];
        Crossing x;
    };
    thread_local std::vector<EdgeCrossing> found;
    found.clear();
    float ranges[3][2];
    for (int a = 0; a < 3; a++) {
        const int u = AxisU(a), v = AxisV(a);
        float t[BrickLines];
        for (int k = 0; k < BrickLines; k++) {
            t[k] = layout.Coord(a, l.origin[a] + k * l.step);
        }
        float* range = ranges[a];
        stock.LineRange(leaf, a, range[0], range[1]);
        for (int j = 0; j < BrickLines; j++) {
            for (int i = 0; i < BrickLines; i++) {
                const int base = i * PointStride[u] + j * PointStride[v];
                const Span* spans = nullptr;
                int n = -1;
                double pu = 0, pv = 0;
                for (int k = 0; k < BrickCells; k++) {
                    const uint8_t in0 = in[base + k * PointStride[a]];
                    if (in0 == in[base + (k + 1) * PointStride[a]]) {
                        continue;
                    }
                    if (n < 0) {
                        n = stock.Line(leaf, a * BrickAxisLines + j * BrickLines + i, spans, scratch);
                        pu = layout.Coord(u, l.origin[u] + i * l.step) - center[u];
                        pv = layout.Coord(v, l.origin[v] + j * l.step) - center[v];
                    }
                    Crossing x = FindCrossing(spans, n, t[k], t[k + 1], in0 != 0, range[0], range[1]);
                    if (x.valid && !HasNormal(x.normal)) {
                        EstimateNormal(stock, leaf, a, i, j, in0 != 0, (float)h, x);
                    }
                    if (measure) {
                        x.thickness = CrossingThickness(stock, a, (float)(pu + center[u]), (float)(pv + center[v]), x, in0 != 0);
                    }
                    g->edges.push_back((uint16_t)EdgeIndex(a, i, j, k));
                    g->crossings.push_back(x);
                    EdgeCrossing e {a, i, j, k, in0 != 0, {0, 0, 0}, x};
                    if (x.valid) {
                        e.p[a] = x.t - center[a];
                        e.p[u] = pu;
                        e.p[v] = pv;
                    }
                    found.push_back(e);
                }
            }
        }
    }

    // Ridges between the lines along an axis: the cells around them get a plane through the
    // ridge, their crossings on different sides of it get separate vertex normals, so the ridge
    // is sharp in the shading like on the machined part
    struct RidgePlane
    {
        double m[3];  // on the ridge
        double d[3];  // across the ridge
    };
    thread_local RidgePlane ridges[Cells][MaxRidges];
    uint8_t nRidges[Cells] = {};
    // the end of material of a line nearest to t of the kind of a crossing, within two cells, if
    // it is on a cut face: its position relative to the center and its normal
    auto cutEnd = [&](int a, int i, int j, bool in0, double t, double p[3], float n[3]) {
        if (i < 0 || i >= BrickLines || j < 0 || j >= BrickLines) {
            return false;
        }
        const Span* spans;
        Span lineScratch;
        const int count = stock.Line(leaf, a * BrickAxisLines + j * BrickLines + i, spans, lineScratch);
        double best = 2 * h;
        float end = 0;
        uint32_t normal = 0;
        bool any = false;
        for (int s = 0; s < count; s++) {
            const float e = in0 ? spans[s].hi : spans[s].lo;
            if (e <= ranges[a][0] || e >= ranges[a][1] || std::abs(e - t) >= best) {
                continue;
            }
            best = std::abs(e - t);
            end = e;
            normal = in0 ? spans[s].nhi : spans[s].nlo;
            any = true;
        }
        if (!any || !HasNormal(normal) || !IsCutFace(normal)) {
            return false;
        }
        const int u = AxisU(a), v = AxisV(a);
        p[a] = end - center[a];
        p[u] = layout.Coord(u, l.origin[u] + i * l.step) - center[u];
        p[v] = layout.Coord(v, l.origin[v] + j * l.step) - center[v];
        UnpackNormal(normal, n);
        return true;
    };
    // how much the surface bends convex from a to b, in radians
    auto bend = [](const double* pa, const float* na, const double* pb, const float* nb) {
        double len = 0, dot = 0;
        for (int k = 0; k < 3; k++) {
            const double d = pb[k] - pa[k];
            len += d * d;
            dot += (nb[k] - na[k]) * d;
        }
        return len > 0 ? dot / std::sqrt(len) : 0.0;
    };
    for (const EdgeCrossing& e : found) {
        if (!e.x.valid || !HasNormal(e.x.normal) || !IsCutFace(e.x.normal)) {
            continue;
        }
        float n[3];
        UnpackNormal(e.x.normal, n);
        const int u = AxisU(e.a), v = AxisV(e.a);
        const double t = e.p[e.a] + center[e.a];
        // towards the next line along u and along v
        for (int dir = 0; dir < 2; dir++) {
            const int di = dir == 0 ? 1 : 0, dj = 1 - di;
            double q[3], before[3], after[3];
            float nq[3], nb[3], na[3];
            if (!cutEnd(e.a, e.i + di, e.j + dj, e.in0, t, q, nq)) {
                continue;
            }
            const double across = bend(e.p, n, q, nq);
            if (across < RidgeBend) {
                continue;
            }
            // compared with the bend on either side, unknown without one
            double side = 0;
            bool known = false;
            if (cutEnd(e.a, e.i - di, e.j - dj, e.in0, t, before, nb)) {
                side = std::max(side, bend(before, nb, e.p, n));
                known = true;
            }
            if (cutEnd(e.a, e.i + 2 * di, e.j + 2 * dj, e.in0, q[e.a] + center[e.a], after, na)) {
                side = std::max(side, bend(q, nq, after, na));
                known = true;
            }
            if (!known || across <= RidgeRatio * side) {
                continue;
            }
            RidgePlane plane;
            double len = 0;
            for (int k = 0; k < 3; k++) {
                plane.m[k] = (e.p[k] + q[k]) / 2;
                plane.d[k] = q[k] - e.p[k];
                len += plane.d[k] * plane.d[k];
            }
            len = std::sqrt(len);
            for (double& d : plane.d) {
                d /= len;
            }
            // the cells between the two lines, at the heights of both crossings
            const int along = dir == 0 ? u : v, other = dir == 0 ? v : u;
            const int kq = (int)std::floor((q[e.a] + center[e.a] - layout.Coord(e.a, l.origin[e.a])) / h);
            for (const int k : {e.k, kq}) {
                for (int o = -1; o <= 0; o++) {
                    int c[3];
                    c[e.a] = k;
                    c[along] = dir == 0 ? e.i : e.j;
                    c[other] = (dir == 0 ? e.j : e.i) + o;
                    if (c[0] < 0 || c[0] >= BrickCells || c[1] < 0 || c[1] >= BrickCells || c[2] < 0
                        || c[2] >= BrickCells) {
                        continue;
                    }
                    const int cell = CellIndex(c);
                    bool known = false;
                    for (int r = 0; r < nRidges[cell] && !known; r++) {
                        const RidgePlane& o2 = ridges[cell][r];
                        double dd = 0, dm = 0;
                        for (int m = 0; m < 3; m++) {
                            dd += o2.d[m] * plane.d[m];
                            dm += (plane.m[m] - o2.m[m]) * o2.d[m];
                        }
                        known = dd > 0.9 && std::abs(dm) < 0.3 * h;
                    }
                    if (!known && nRidges[cell] < MaxRidges) {
                        ridges[cell][nRidges[cell]++] = plane;
                    }
                }
            }
        }
    }

    // planes of the cells, the crossings of the edges around them
    for (const EdgeCrossing& e : found) {
        const int u = AxisU(e.a), v = AxisV(e.a);
        for (int dv = -1; dv <= 0; dv++) {
            for (int du = -1; du <= 0; du++) {
                int c[3];
                c[e.a] = e.k;
                c[u] = e.i + du;
                c[v] = e.j + dv;
                if (c[u] < 0 || c[u] >= BrickCells || c[v] < 0 || c[v] >= BrickCells) {
                    continue;
                }
                const int cell = CellIndex(c);
                Qef& q = qefOf(cell);
                if (!e.x.valid) {
                    q.incomplete = true;
                    continue;
                }
                uint8_t side = 0;
                for (int r = 0; r < nRidges[cell]; r++) {
                    const RidgePlane& plane = ridges[cell][r];
                    double d = 0;
                    for (int m = 0; m < 3; m++) {
                        d += (e.p[m] - plane.m[m]) * plane.d[m];
                    }
                    side |= (uint8_t)((d > 0 ? 1 : 0) << r);
                }
                q.ridge |= nRidges[cell] > 0;
                q.Add(e.p, e.x, side);
            }
        }
    }

    // merge cubes of 8 cubes of half the size where one vertex fits and the shape stays the
    // same. qef: QEF of each cube with a surface, by level and cube index
    int16_t cubeQef[BrickLog + 1][Cells];
    std::copy(std::begin(qIndex), std::end(qIndex), cubeQef[0]);
    bool merged[BrickLog + 1][Cells] = {};
    for (int log = 1; log <= BrickLog; log++) {
        const int size = 1 << log, half = size / 2, n = BrickCells >> log;
        for (int gi = 0; gi < n * n * n; gi++) {
            const int gc[3] = {gi % n, gi / n % n, gi / (n * n)};
            const int g0[3] = {gc[0] * size, gc[1] * size, gc[2] * size};
            cubeQef[log][gi] = -1;
            // the 8 sub-cubes
            const int hn = n * 2;
            bool children = true, surface = false;
            int child[8];
            for (int ch = 0; ch < 8; ch++) {
                const int cc[3] = {gc[0] * 2 + (ch & 1), gc[1] * 2 + ((ch >> 1) & 1), gc[2] * 2 + (ch >> 2)};
                child[ch] = cc[0] + hn * (cc[1] + hn * cc[2]);
                children &= log == 1 || merged[log - 1][child[ch]];
                surface |= cubeQef[log - 1][child[ch]] >= 0;
            }
            if (!children) {
                continue;
            }
            if (surface) {
                if (!SafeToMerge(in, g0, half)) {
                    continue;
                }
                Qef sum;
                sum.Clear();
                for (int ch = 0; ch < 8; ch++) {
                    if (cubeQef[log - 1][child[ch]] >= 0) {
                        sum.Merge(qefs[cubeQef[log - 1][child[ch]]]);
                    }
                }
                if (sum.incomplete || sum.ridge || sum.count == 0 || sum.Mixed()
                    || sum.NormalSpread() > MergeNormalTolerance) {
                    continue;
                }
                double x[3];
                sum.Solve(x);
                bool inside = true;
                for (int k = 0; k < 3; k++) {
                    const double lo = layout.Coord(k, l.origin[k] + g0[k] * l.step) - center[k];
                    const double hi = layout.Coord(k, l.origin[k] + (g0[k] + size) * l.step) - center[k];
                    inside &= x[k] >= lo - 1e-6 * h && x[k] <= hi + 1e-6 * h;
                }
                const double tolerance = MergeTolerance * h;
                if (!inside || sum.Error(x) > tolerance * tolerance * sum.count) {
                    continue;
                }
                cubeQef[log][gi] = (int16_t)qefs.size();
                qefs.push_back(sum);
            }
            merged[log][gi] = true;
            if (log == 1) {
                g->merged2 |= uint64_t(1) << gi;
            }
            else if (log == 2) {
                g->merged4 |= (uint8_t)(1 << gi);
            }
            else {
                g->merged8 = true;
            }
        }
    }

    // a vertex for every cube with a surface
    for (int cell = 0; cell < Cells; cell++) {
        int c[3];
        CellCoords(cell, c);
        const int log = g->CubeLog(c);
        const int size = 1 << log;
        if (c[0] % size || c[1] % size || c[2] % size) {
            continue;  // not the first cell of its cube
        }
        const int n = BrickCells >> log;
        const int q = cubeQef[log][(c[0] >> log) + n * ((c[1] >> log) + n * (c[2] >> log))];
        if (q < 0) {
            continue;
        }
        const Qef& qef = qefs[q];
        double pos[3];
        double lo[3], hi[3];
        for (int k = 0; k < 3; k++) {
            lo[k] = layout.Coord(k, l.origin[k] + c[k] * l.step) - center[k];
            hi[k] = layout.Coord(k, l.origin[k] + (c[k] + size) * l.step) - center[k];
            pos[k] = (lo[k] + hi[k]) / 2;  // no crossings found: the middle of the cube
        }
        if (qef.points > 0) {
            qef.Solve(pos);
        }
        LeafGeometry::Vertex vertex {};
        for (int k = 0; k < 3; k++) {
            vertex.p[k] = (float)(center[k] + std::clamp(pos[k], lo[k], hi[k]));
        }
        vertex.cell = (uint16_t)cell;
        vertex.firstCluster = (uint16_t)g->clusters.size();
        vertex.clusters = (uint8_t)qef.nClusters;
        g->clusters.insert(g->clusters.end(), qef.clusters, qef.clusters + qef.nClusters);
        g->vertices.push_back(vertex);

        // crossings of the edges of merged cubes, the mesh only uses the cached surface
        if (log == 0) {
            continue;
        }
        for (int a = 0; a < 3; a++) {
            const int u = AxisU(a), v = AxisV(a);
            float range[2];
            stock.LineRange(leaf, a, range[0], range[1]);
            for (int ev = 0; ev < 2; ev++) {
                for (int eu = 0; eu < 2; eu++) {
                    const int i = c[u] + eu * size, j = c[v] + ev * size;
                    int p0[3];
                    p0[a] = c[a];
                    p0[u] = i;
                    p0[v] = j;
                    const int point0 = PointIndex(p0);
                    const bool in0 = g->Inside(point0);
                    if (in0 == g->Inside(point0 + size * PointStride[a])) {
                        continue;
                    }
                    const Span* spans;
                    const int n = stock.Line(leaf, a * BrickAxisLines + j * BrickLines + i, spans, scratch);
                    Crossing x = FindCrossing(
                        spans,
                        n,
                        layout.Coord(a, l.origin[a] + c[a] * l.step),
                        layout.Coord(a, l.origin[a] + (c[a] + size) * l.step),
                        in0,
                        range[0],
                        range[1]
                    );
                    if (measure) {
                        x.thickness = CrossingThickness(
                            stock,
                            a,
                            layout.Coord(u, l.origin[u] + i * l.step),
                            layout.Coord(v, l.origin[v] + j * l.step),
                            x,
                            in0
                        );
                    }
                    g->coarse.emplace_back(LeafGeometry::CoarseKey(a, i, j, c[a], log), x);
                }
            }
        }
    }
    std::sort(g->coarse.begin(), g->coarse.end(), [](const auto& x, const auto& y) {
        return x.first < y.first;
    });
    return g;
}

class Mesher
{
public:
    Mesher(const DexelStock& stock, const DexelMeshCache& cache, DexelMesh& mesh)
        : mCache(cache)
        , mLayout(stock.Layout())
        , mMesh(mesh)
    {}

    void Build(const std::vector<int>& leaves);

private:
    // A cube of merged cells: its leaf and first cell, and its size in cells of the finest level
    struct Corner
    {
        int leaf;
        int cell;
        int size;
        bool operator==(const Corner& o) const
        {
            return leaf == o.leaf && cell == o.cell;
        }
    };
    // Vertex of a cube with its clusters
    struct CubeVertex
    {
        const float* p;
        const Cluster* clusters;
        int count;
        uint64_t key;
    };
    // Vertex of a cube without a surface of its own
    struct ExtraVertex
    {
        float p[3];
        std::vector<Cluster> clusters;
    };
    // A quad in a plane perpendicular to an axis, at grid position (u, v) of its cube size
    struct FlatQuad
    {
        int u, v;
        Corner corners[4];
    };
    // Planes: axis, direction of the normal, position, cube size, cut flag and operation tag
    using PlaneKey = std::tuple<int, bool, float, int, bool, int>;

    Corner CubeAt(int leaf, const int c[3]) const;
    CubeVertex Vertex(const Corner& corner);
    void AddFaceCrossings(const Corner& corner, Qef& qef, const double origin[3]);
    // A vertex of the mesh at a cube vertex, with the cluster of the normal. The operation tag and
    // the thickness are those of the cluster, or the given ones without a cluster.
    uint32_t MeshVertex(const CubeVertex& v, const float normal[3], int operation, float thickness);
    void Polygon(const Corner* corners, int count, bool flip, const Crossing& crossing);
    bool IsFlat(const Corner& corner, int a, float t);
    void MergeFlat(const PlaneKey& key, std::vector<FlatQuad>& quads);

    const DexelMeshCache& mCache;
    const DexelLayout& mLayout;
    DexelMesh& mMesh;
    std::unordered_map<uint64_t, ExtraVertex> mExtra;
    std::unordered_map<uint64_t, uint32_t> mIndex;
    std::map<PlaneKey, std::vector<FlatQuad>> mFlat;
};

Mesher::Corner Mesher::CubeAt(int leaf, const int c[3]) const
{
    const LeafGeometry* g = mCache.Get(leaf);
    const int log = g ? g->CubeLog(c) : BrickLog;
    const int mask = ~((1 << log) - 1);
    const int first[3] = {c[0] & mask, c[1] & mask, c[2] & mask};
    return {leaf, CellIndex(first), mLayout.Leaf(leaf).step << log};
}

// Crossings on the faces of a cube shared with finer leaves, which the cube doesn't see itself
void Mesher::AddFaceCrossings(const Corner& corner, Qef& qef, const double origin[3])
{
    const LayoutLeaf& l = mLayout.Leaf(corner.leaf);
    int c[3];
    CellCoords(corner.cell, c);
    int f0[3];
    for (int k = 0; k < 3; k++) {
        f0[k] = l.origin[k] + c[k] * l.step;
    }
    const int extent = corner.size;
    std::vector<int> seen;
    for (int a = 0; a < 3; a++) {
        for (int side = 0; side < 2; side++) {
            const int plane = f0[a] + side * extent;
            // the neighbors along the face, finer leaves can be smaller than the face
            seen.clear();
            for (int sv = 0; sv < extent; sv += std::max(1, extent / 4)) {
                for (int su = 0; su < extent; su += std::max(1, extent / 4)) {
                    int probe[3];
                    probe[a] = side ? plane : plane - 1;
                    probe[AxisU(a)] = f0[AxisU(a)] + su;
                    probe[AxisV(a)] = f0[AxisV(a)] + sv;
                    const int neighbor = mLayout.FindLeaf(probe[0], probe[1], probe[2]);
                    if (neighbor < 0 || mLayout.Leaf(neighbor).level <= l.level
                        || std::find(seen.begin(), seen.end(), neighbor) != seen.end()) {
                        continue;
                    }
                    seen.push_back(neighbor);
                    const LayoutLeaf& n = mLayout.Leaf(neighbor);
                    const LeafGeometry* ng = mCache.Get(neighbor);
                    if (!ng) {
                        continue;
                    }
                    // edges of the neighbor in the plane, inside the face of the cube
                    const int pa = (plane - n.origin[a]) / n.step;
                    for (int b = 0; b < 3; b++) {
                        if (b == a) {
                            continue;
                        }
                        const int other = 3 - a - b;
                        for (int m = 0; m < BrickLines; m++) {
                            const int fm = n.origin[other] + m * n.step;
                            if (fm < f0[other] || fm > f0[other] + extent) {
                                continue;
                            }
                            for (int k = 0; k < BrickCells; k++) {
                                const int fk = n.origin[b] + k * n.step;
                                if (fk < f0[b] || fk + n.step > f0[b] + extent) {
                                    continue;
                                }
                                // line along b at coordinate pa along a and m along 'other'
                                const int i = AxisU(b) == a ? pa : m;
                                const int j = AxisU(b) == a ? m : pa;
                                const Crossing* x = ng->CrossingAt(EdgeIndex(b, i, j, k));
                                if (!x || !x->valid) {
                                    continue;
                                }
                                double p[3];
                                p[b] = x->t - origin[b];
                                p[AxisU(b)] = mLayout.Coord(AxisU(b), n.origin[AxisU(b)] + i * n.step) - origin[AxisU(b)];
                                p[AxisV(b)] = mLayout.Coord(AxisV(b), n.origin[AxisV(b)] + j * n.step) - origin[AxisV(b)];
                                qef.Add(p, *x);
                            }
                        }
                    }
                }
            }
        }
    }
}

Mesher::CubeVertex Mesher::Vertex(const Corner& corner)
{
    const uint64_t key = ((uint64_t)corner.leaf << 16) | ((uint64_t)corner.cell << 3);
    if (const LeafGeometry* g = mCache.Get(corner.leaf)) {
        if (const LeafGeometry::Vertex* v = g->VertexAt(corner.cell)) {
            return {v->p, g->clusters.data() + v->firstCluster, v->clusters, key};
        }
    }
    // no change on the edges of the cube, but on the faces it shares with finer leaves
    auto it = mExtra.find(key);
    if (it == mExtra.end()) {
        const LayoutLeaf& l = mLayout.Leaf(corner.leaf);
        int c[3];
        CellCoords(corner.cell, c);
        double lo[3], hi[3], center[3];
        for (int k = 0; k < 3; k++) {
            lo[k] = mLayout.Coord(k, l.origin[k] + c[k] * l.step);
            hi[k] = mLayout.Coord(k, l.origin[k] + c[k] * l.step + corner.size);
            center[k] = (lo[k] + hi[k]) / 2;
        }
        Qef qef;
        qef.Clear();
        AddFaceCrossings(corner, qef, center);
        double pos[3] = {0, 0, 0};
        if (qef.points > 0) {
            qef.Solve(pos);
        }
        ExtraVertex extra;
        for (int k = 0; k < 3; k++) {
            extra.p[k] = (float)std::clamp(center[k] + pos[k], lo[k], hi[k]);
        }
        extra.clusters.assign(qef.clusters, qef.clusters + qef.nClusters);
        it = mExtra.emplace(key, std::move(extra)).first;
    }
    return {it->second.p, it->second.clusters.data(), (int)it->second.clusters.size(), key};
}

uint32_t Mesher::MeshVertex(const CubeVertex& v, const float normal[3], int operation, float thickness)
{
    // the cluster of the crossing normal, so sharp edges keep separate normals
    int best = -1;
    float bestDot = 0.5f;
    for (int i = 0; i < v.count; i++) {
        const float* s = v.clusters[i].n;
        const float len = std::sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
        const float dot = (s[0] * normal[0] + s[1] * normal[1] + s[2] * normal[2]) / len;
        if (dot > bestDot) {
            bestDot = dot;
            best = i;
        }
    }
    auto add = [&](const float* n, int tag, float t) {
        const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        const float s = len > 0 ? 127 / len : 0;
        mMesh.vertices.push_back({
            v.p[0],
            v.p[1],
            v.p[2],
            (int8_t)std::lround(n[0] * s),
            (int8_t)std::lround(n[1] * s),
            (int8_t)std::lround(n[2] * s),
            (uint8_t)tag,
            t,
        });
        return (uint32_t)mMesh.vertices.size() - 1;
    };
    if (best < 0) {
        return add(normal, operation, thickness);
    }
    const auto [it, inserted] = mIndex.emplace(v.key | (uint64_t)best, 0);
    if (inserted) {
        const Cluster& c = v.clusters[best];
        it->second = add(c.n, c.operation, c.thickness);
    }
    return it->second;
}

void Mesher::Polygon(const Corner* corners, int count, bool flip, const Crossing& crossing)
{
    CubeVertex v[4];
    for (int i = 0; i < count; i++) {
        v[flip ? count - 1 - i : i] = Vertex(corners[i]);
    }
    // normal of the crossing, or of the polygon
    float normal[3];
    bool cut = true;
    if (crossing.valid && HasNormal(crossing.normal)) {
        UnpackNormal(crossing.normal, normal);
        cut = IsCutFace(crossing.normal);
    }
    else {
        const float* p0 = v[0].p;
        const float* p1 = v[1].p;
        const float* p2 = v[2].p;
        const float* p3 = v[count - 1].p;
        const float d1[3] = {p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]};
        const float d2[3] = {p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]};
        normal[0] = d1[1] * d2[2] - d1[2] * d2[1];
        normal[1] = d1[2] * d2[0] - d1[0] * d2[2];
        normal[2] = d1[0] * d2[1] - d1[1] * d2[0];
        const float len = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
        if (len <= 0) {
            return;
        }
        for (float& n : normal) {
            n /= len;
        }
        if (crossing.valid) {
            cut = IsCutFace(crossing.normal);
        }
        else {
            int cuts = 0, stocks = 0;
            for (int i = 0; i < count; i++) {
                for (int c = 0; c < v[i].count; c++) {
                    cuts += v[i].clusters[c].cut;
                    stocks += v[i].clusters[c].stock;
                }
            }
            cut = cuts >= stocks;
        }
    }
    const int operation = crossing.valid ? OperationTagOf(crossing.normal) : 0;
    uint32_t index[4];
    for (int i = 0; i < count; i++) {
        index[i] = MeshVertex(v[i], normal, operation, crossing.thickness);
    }
    auto& indices = cut ? mMesh.cutIndices : mMesh.stockIndices;
    if (count == 3) {
        indices.insert(indices.end(), {index[0], index[1], index[2]});
        return;
    }
    // split along the diagonal that keeps both triangles facing along the normal, a twisted
    // quad would show a triangle from behind otherwise
    auto facing = [&](int a, int b, int c) {
        const float* pa = v[a].p;
        const float* pb = v[b].p;
        const float* pc = v[c].p;
        const float e1[3] = {pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]};
        const float e2[3] = {pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]};
        const float n[3] = {
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        };
        const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        // degenerate triangles don't show
        return len > 0 ? (n[0] * normal[0] + n[1] * normal[1] + n[2] * normal[2]) / len : 1.0f;
    };
    const float splitA = std::min(facing(0, 1, 2), facing(0, 2, 3));
    const float splitB = std::min(facing(0, 1, 3), facing(1, 2, 3));
    if (splitA >= 0 || splitB >= 0) {
        if (splitA >= splitB) {
            indices.insert(indices.end(), {index[0], index[1], index[2], index[0], index[2], index[3]});
        }
        else {
            indices.insert(indices.end(), {index[0], index[1], index[3], index[1], index[2], index[3]});
        }
        return;
    }
    // around the middle of the quad
    DexelVertex middle {0, 0, 0, 0, 0, 0, (uint8_t)operation, crossing.thickness};
    for (int i = 0; i < 4; i++) {
        middle.x += v[i].p[0] / 4;
        middle.y += v[i].p[1] / 4;
        middle.z += v[i].p[2] / 4;
    }
    middle.nx = (int8_t)std::lround(normal[0] * 127);
    middle.ny = (int8_t)std::lround(normal[1] * 127);
    middle.nz = (int8_t)std::lround(normal[2] * 127);
    const auto m = (uint32_t)mMesh.vertices.size();
    mMesh.vertices.push_back(middle);
    for (int i = 0; i < 4; i++) {
        indices.insert(indices.end(), {index[i], index[(i + 1) % 4], m});
    }
}

// True if the vertex of the cube is on the plane at t perpendicular to axis a, and only sees
// that plane
bool Mesher::IsFlat(const Corner& corner, int a, float t)
{
    const LeafGeometry* g = mCache.Get(corner.leaf);
    const LeafGeometry::Vertex* v = g ? g->VertexAt(corner.cell) : nullptr;
    if (!v || v->clusters != 1 || v->p[a] != t) {
        return false;
    }
    const float* n = g->clusters[v->firstCluster].n;
    const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    return std::abs(n[a]) > 0.99999f * len;
}

// Merge the quads of a plane into rectangles. A rectangle keeps all vertices on its border, so it
// connects to its neighbors without cracks.
void Mesher::MergeFlat(const PlaneKey& key, std::vector<FlatQuad>& quads)
{
    const int axis = std::get<0>(key);
    const bool positive = std::get<1>(key);
    const bool cut = std::get<4>(key);
    std::sort(quads.begin(), quads.end(), [](const FlatQuad& x, const FlatQuad& y) {
        return x.v != y.v ? x.v < y.v : x.u < y.u;
    });
    std::map<std::pair<int, int>, int> index;
    for (int i = 0; i < (int)quads.size(); i++) {
        index[{quads[i].u, quads[i].v}] = i;
    }
    // cubes at the corners of the quads, by their grid position: quad (u, v) has the cubes
    // (u - 1, v - 1), (u, v - 1), (u, v), (u - 1, v)
    std::map<std::pair<int, int>, Corner> cubes;
    static const int du[4] = {-1, 0, 0, -1};
    static const int dv[4] = {-1, -1, 0, 0};
    for (const FlatQuad& q : quads) {
        for (int c = 0; c < 4; c++) {
            cubes[{q.u + du[c], q.v + dv[c]}] = q.corners[c];
        }
    }
    float normal[3] = {0, 0, 0};
    normal[axis] = positive ? 1.0f : -1.0f;
    auto& indices = cut ? mMesh.cutIndices : mMesh.stockIndices;
    std::vector<uint8_t> used(quads.size(), 0);
    std::vector<uint32_t> border;
    auto free = [&](int u, int v) {
        const auto it = index.find({u, v});
        return it != index.end() && !used[it->second];
    };
    for (int i = 0; i < (int)quads.size(); i++) {
        if (used[i]) {
            continue;
        }
        const int u0 = quads[i].u, v0 = quads[i].v;
        int u1 = u0;
        while (free(u1 + 1, v0)) {
            u1++;
        }
        int v1 = v0;
        while (true) {
            bool row = true;
            for (int u = u0; u <= u1 && row; u++) {
                row = free(u, v1 + 1);
            }
            if (!row) {
                break;
            }
            v1++;
        }
        for (int v = v0; v <= v1; v++) {
            for (int u = u0; u <= u1; u++) {
                used[index[{u, v}]] = 1;
            }
        }
        // border cubes counter clockwise in (u, v), around +a
        border.clear();
        auto add = [&](int u, int v) {
            border.push_back(MeshVertex(Vertex(cubes[{u, v}]), normal, std::get<5>(key), ThicknessUnknown));
        };
        for (int u = u0 - 1; u < u1; u++) {
            add(u, v0 - 1);
        }
        for (int v = v0 - 1; v < v1; v++) {
            add(u1, v);
        }
        for (int u = u1; u > u0 - 1; u--) {
            add(u, v1);
        }
        for (int v = v1; v > v0 - 1; v--) {
            add(u0 - 1, v);
        }
        if (!positive) {
            std::reverse(border.begin(), border.end());
        }
        // fan from a corner, the triangles along its two sides have no area
        for (size_t k = 1; k + 1 < border.size(); k++) {
            indices.insert(indices.end(), {border[0], border[k], border[k + 1]});
        }
    }
}

void Mesher::Build(const std::vector<int>& leaves)
{
    static const int du[4] = {-1, 0, 0, -1};
    static const int dv[4] = {-1, -1, 0, 0};
    // the edge belongs to the first of these cubes of its size
    static const int priority[4] = {2, 3, 1, 0};
    for (int leaf : leaves) {
        const LeafGeometry* g = mCache.Get(leaf);
        if (!g) {
            continue;
        }
        const LayoutLeaf& l = mLayout.Leaf(leaf);
        for (const LeafGeometry::Vertex& vertex : g->vertices) {
            const int cell = vertex.cell;
            int c[3];
            CellCoords(cell, c);
            const int log = g->CubeLog(c);
            const int size = 1 << log;
            const Corner self {leaf, cell, l.step << log};
            // the 12 edges of the cube
            for (int a = 0; a < 3; a++) {
                const int u = AxisU(a), v = AxisV(a);
                for (int ev = 0; ev < 2; ev++) {
                    for (int eu = 0; eu < 2; eu++) {
                        const int i = c[u] + eu * size, j = c[v] + ev * size;
                        int p0[3];
                        p0[a] = c[a];
                        p0[u] = i;
                        p0[v] = j;
                        const int point0 = PointIndex(p0);
                        const bool in0 = g->Inside(point0);
                        if (in0 == g->Inside(point0 + size * PointStride[a])) {
                            continue;
                        }
                        // the cubes around the edge, counter clockwise around the axis, found by
                        // the cells next to its start
                        Corner corners[4];
                        bool skip = false;
                        for (int q = 0; q < 4 && !skip; q++) {
                            int n[3];
                            n[a] = c[a];
                            n[u] = i + du[q];
                            n[v] = j + dv[q];
                            if (n[u] >= 0 && n[u] < BrickCells && n[v] >= 0 && n[v] < BrickCells) {
                                corners[q] = CubeAt(leaf, n);
                            }
                            else {
                                int f[3];
                                for (int m = 0; m < 3; m++) {
                                    f[m] = l.origin[m] + n[m] * l.step;
                                }
                                const int other = mLayout.FindLeaf(f[0], f[1], f[2]);
                                if (other < 0) {
                                    skip = true;
                                    break;
                                }
                                const LayoutLeaf& o = mLayout.Leaf(other);
                                int local[3];
                                for (int m = 0; m < 3; m++) {
                                    local[m] = (f[m] - o.origin[m]) / o.step;
                                }
                                corners[q] = CubeAt(other, local);
                            }
                            // a smaller cube divides the edge, its parts make the polygons
                            skip = corners[q].size < self.size;
                        }
                        if (skip) {
                            continue;
                        }
                        Corner owner {-1, -1, 0};
                        for (int q : priority) {
                            if (corners[q].size == self.size) {
                                owner = corners[q];
                                break;
                            }
                        }
                        if (!(owner == self)) {
                            continue;
                        }
                        // remove repeated larger cubes
                        Corner unique[4];
                        int count = 0;
                        for (int q = 0; q < 4; q++) {
                            if (corners[q] == corners[(q + 3) % 4]) {
                                continue;
                            }
                            unique[count++] = corners[q];
                        }
                        if (count < 3) {
                            continue;
                        }
                        Crossing x {0, 0, false};
                        if (size == 1) {
                            if (const Crossing* fine = g->CrossingAt(EdgeIndex(a, i, j, c[a]))) {
                                x = *fine;
                            }
                        }
                        else {
                            x = g->CoarseCrossingAt(LeafGeometry::CoarseKey(a, i, j, c[a], log));
                        }
                        // normal towards +a if the material is before the crossing
                        const bool flip = !in0;
                        // quads in planes perpendicular to the axis are merged later
                        if (count == 4 && x.valid && HasNormal(x.normal) && !mCache.MeasuresThickness()) {
                            float n[3];
                            UnpackNormal(x.normal, n);
                            if (std::abs(n[a]) > 0.99999f && (n[a] > 0) != flip) {
                                bool flat = true;
                                for (int q = 0; q < 4 && flat; q++) {
                                    flat = corners[q].size == self.size && IsFlat(corners[q], a, x.t);
                                }
                                if (flat) {
                                    const PlaneKey key {
                                        a,
                                        n[a] > 0,
                                        x.t,
                                        self.size,
                                        IsCutFace(x.normal),
                                        OperationTagOf(x.normal),
                                    };
                                    FlatQuad quad;
                                    quad.u = (l.origin[u] + i * l.step) / self.size;
                                    quad.v = (l.origin[v] + j * l.step) / self.size;
                                    std::copy(corners, corners + 4, quad.corners);
                                    mFlat[key].push_back(quad);
                                    continue;
                                }
                            }
                        }
                        Polygon(unique, count, flip, x);
                    }
                }
            }
        }
    }
    for (auto& [key, quads] : mFlat) {
        MergeFlat(key, quads);
    }
}

}  // namespace

DexelMeshCache::DexelMeshCache() = default;
DexelMeshCache::~DexelMeshCache() = default;

void DexelMeshCache::Reset(int leaves)
{
    mLeaves.clear();
    mLeaves.resize(leaves);
    mState.assign(leaves, Missing);
}

void DexelMeshCache::Invalidate(int leaf)
{
    mLeaves[leaf].reset();
    mState[leaf] = Missing;
}

void DexelMeshCache::Update(const DexelStock& stock, const std::vector<int>& leaves, ThreadPool& pool)
{
    std::vector<int> missing;
    for (int leaf : leaves) {
        if (mState[leaf] == Missing) {
            missing.push_back(leaf);
            mState[leaf] = Empty;  // once
        }
    }
    if (missing.empty()) {
        return;
    }
    // leaves take very different times, the threads take the next one when they are done
    std::atomic<size_t> next {0};
    pool.Run([&](int) {
        while (true) {
            const size_t i = next.fetch_add(1, std::memory_order_relaxed);
            if (i >= missing.size()) {
                break;
            }
            const int leaf = missing[i];
            bool solid = false;
            mLeaves[leaf] = ComputeLeaf(stock, leaf, solid, mMeasureThickness);
            mState[leaf] = mLeaves[leaf] ? Surface : solid ? Solid : Empty;
        }
    });
}

void DexelMeshUnits::Reset(const DexelStock& stock)
{
    mLayout = stock.LayoutRef();
    mCache.Reset(mLayout->LeafCount());
    mMeshes.assign(mLayout->UnitCount(), DexelMesh());
    mQueue.clear();
    mQueuedAt.clear();
    mFirst = 0;
    mQueued.assign(mLayout->LeafCount(), 0);
}

void DexelMeshUnits::SetMeasureThickness(bool b)
{
    if (b == mCache.MeasuresThickness()) {
        return;
    }
    mCache.SetMeasureThickness(b);
    if (!mLayout) {
        return;
    }
    for (int leaf = 0; leaf < mLayout->LeafCount(); leaf++) {
        if (!mQueued[leaf]) {
            mQueued[leaf] = 1;
            mQueue.push_back(leaf);
            mQueuedAt.push_back(mUpdates);
        }
    }
}

std::vector<int> DexelMeshUnits::Update(DexelStock& stock, ThreadPool& pool, double budgetMs)
{
    std::vector<int> updated;
    if (!mLayout) {
        return updated;
    }
    mUpdates++;
    mLastDelay = 0;
    for (int leaf : stock.TakeDirtyLeaves()) {
        if (!mQueued[leaf]) {
            mQueued[leaf] = 1;
            mQueue.push_back(leaf);
            mQueuedAt.push_back(mUpdates);
        }
    }
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::duration<double, std::milli>(budgetMs);
    std::vector<uint8_t> inBatch(mMeshes.size(), 0);
    std::vector<int> units, leaves, group;
    // batches of changed leaves, each completely: the new surface of the leaves and all units
    // depending on them. A batch takes what fits in the remaining time at the measured speed,
    // units depending on several of its leaves are built once. At least one batch is done, so
    // the mesh progresses with any budget.
    for (bool first = true; mFirst < mQueue.size() && (first || std::chrono::steady_clock::now() < deadline);
         first = false) {
        const auto start = std::chrono::steady_clock::now();
        const double remaining = std::chrono::duration<double, std::milli>(deadline - start).count();
        const size_t batchUnits = std::max((size_t)pool.Size(), (size_t)(remaining / mMsPerUnit));
        units.clear();
        leaves.clear();
        // Neighboring leaves share the grid points on their border, which both compute from their
        // own copy of the dexels there, so changed neighbors are updated together: whole groups
        // of touching changed leaves.
        while (mFirst < mQueue.size() && units.size() < batchUnits) {
            mLastDelay = std::max(mLastDelay, mUpdates - mQueuedAt[mFirst]);
            const int seed = mQueue[mFirst++];
            if (!mQueued[seed]) {
                continue;  // done with an earlier group
            }
            mQueued[seed] = 0;
            group.assign(1, seed);
            for (size_t g = 0; g < group.size(); g++) {
                const int leaf = group[g];
                mCache.Invalidate(leaf);
                for (int unit : mLayout->AffectedUnits(leaf)) {
                    if (!inBatch[unit]) {
                        inBatch[unit] = 1;
                        units.push_back(unit);
                    }
                }
                for (int neighbor : mLayout->Neighbors(leaf)) {
                    if (mQueued[neighbor]) {
                        mQueued[neighbor] = 0;
                        group.push_back(neighbor);
                    }
                }
            }
        }
        for (int unit : units) {
            const auto& needed = mLayout->UnitDependencies(unit);
            leaves.insert(leaves.end(), needed.begin(), needed.end());
        }
        mCache.Update(stock, leaves, pool);
        std::atomic<size_t> next {0};
        pool.Run([&](int) {
            while (true) {
                const size_t i = next++;
                if (i >= units.size()) {
                    break;
                }
                BuildDexelMesh(stock, mCache, mLayout->UnitLeaves(units[i]), mMeshes[units[i]]);
            }
        });
        for (int unit : units) {
            inBatch[unit] = 0;
            updated.push_back(unit);
        }
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        if (units.size() >= 4) {
            mMsPerUnit = 0.7 * mMsPerUnit + 0.3 * std::max(1e-4, ms / (double)units.size());
        }
    }
    // skip the leaves done with groups
    while (mFirst < mQueue.size() && !mQueued[mQueue[mFirst]]) {
        mFirst++;
    }
    if (mFirst == mQueue.size()) {
        mQueue.clear();
        mQueuedAt.clear();
        mFirst = 0;
    }
    std::sort(updated.begin(), updated.end());
    updated.erase(std::unique(updated.begin(), updated.end()), updated.end());
    return updated;
}

void BuildDexelMesh(
    const DexelStock& stock,
    const DexelMeshCache& cache,
    const std::vector<int>& leaves,
    DexelMesh& mesh
)
{
    mesh.vertices.clear();
    mesh.stockIndices.clear();
    mesh.cutIndices.clear();
    Mesher(stock, cache, mesh).Build(leaves);
}

void BuildDexelMesh(const DexelStock& stock, const std::vector<int>& leaves, DexelMesh& mesh)
{
    // the leaves and their neighbors
    const DexelLayout& layout = stock.Layout();
    std::vector<int> needed;
    for (int leaf : leaves) {
        for (int unit : layout.AffectedUnits(leaf)) {
            const auto& more = layout.UnitDependencies(unit);
            needed.insert(needed.end(), more.begin(), more.end());
        }
    }
    DexelMeshCache cache;
    cache.Reset(layout.LeafCount());
    ThreadPool pool(1);
    cache.Update(stock, needed, pool);
    BuildDexelMesh(stock, cache, leaves, mesh);
}

}  // namespace CAMSimulator
