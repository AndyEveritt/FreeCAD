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


#include "MaterialModel.h"

#include <algorithm>
#include <cmath>

namespace CAMSimulator::Material
{

namespace
{

float Dot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// Squared distance from p to the triangle, and whether the closest point is inside of it (not on
// an edge or a corner), after Ericson, Real-Time Collision Detection
float TriangleDistance2(const float p[3], const float a[3], const float b[3], const float c[3], bool& interior)
{
    float ab[3], ac[3], ap[3], q[3];
    for (int k = 0; k < 3; k++) {
        ab[k] = b[k] - a[k];
        ac[k] = c[k] - a[k];
        ap[k] = p[k] - a[k];
    }
    interior = false;
    auto dist2 = [&](const float* r) {
        const float d[3] = {p[0] - r[0], p[1] - r[1], p[2] - r[2]};
        return Dot(d, d);
    };
    const float d1 = Dot(ab, ap), d2 = Dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) {
        return dist2(a);
    }
    const float bp[3] = {p[0] - b[0], p[1] - b[1], p[2] - b[2]};
    const float d3 = Dot(ab, bp), d4 = Dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) {
        return dist2(b);
    }
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const float v = d1 / (d1 - d3);
        for (int k = 0; k < 3; k++) {
            q[k] = a[k] + v * ab[k];
        }
        return dist2(q);
    }
    const float cp[3] = {p[0] - c[0], p[1] - c[1], p[2] - c[2]};
    const float d5 = Dot(ab, cp), d6 = Dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) {
        return dist2(c);
    }
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const float w = d2 / (d2 - d6);
        for (int k = 0; k < 3; k++) {
            q[k] = a[k] + w * ac[k];
        }
        return dist2(q);
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        for (int k = 0; k < 3; k++) {
            q[k] = b[k] + w * (c[k] - b[k]);
        }
        return dist2(q);
    }
    const float denom = 1 / (va + vb + vc);
    const float v = vb * denom, w = vc * denom;
    for (int k = 0; k < 3; k++) {
        q[k] = a[k] + ab[k] * v + ac[k] * w;
    }
    interior = true;
    return dist2(q);
}

}  // namespace

ModelDistance::ModelDistance(
    const std::vector<float>& xyz,
    const std::vector<uint32_t>& triangles,
    float range
)
    : mRange(range)
{
    const size_t vertices = xyz.size() / 3;
    float hi[3] = {-1e30f, -1e30f, -1e30f};
    std::fill(mLo, mLo + 3, 1e30f);
    for (size_t t = 0; t + 2 < triangles.size(); t += 3) {
        Triangle tri;
        float* corners[3] = {tri.a, tri.b, tri.c};
        bool valid = true;
        for (int c = 0; c < 3; c++) {
            const uint32_t v = triangles[t + c];
            valid = valid && v < vertices;
            for (int k = 0; valid && k < 3; k++) {
                corners[c][k] = xyz[3 * v + k];
            }
        }
        if (!valid) {
            continue;
        }
        const float u[3] = {tri.b[0] - tri.a[0], tri.b[1] - tri.a[1], tri.b[2] - tri.a[2]};
        const float w[3] = {tri.c[0] - tri.a[0], tri.c[1] - tri.a[1], tri.c[2] - tri.a[2]};
        tri.n[0] = u[1] * w[2] - u[2] * w[1];
        tri.n[1] = u[2] * w[0] - u[0] * w[2];
        tri.n[2] = u[0] * w[1] - u[1] * w[0];
        const float len = std::sqrt(Dot(tri.n, tri.n));
        if (!(len > 0)) {
            continue;
        }
        for (float& c : tri.n) {
            c /= len;
        }
        for (const float* corner : {tri.a, tri.b, tri.c}) {
            for (int k = 0; k < 3; k++) {
                mLo[k] = std::min(mLo[k], corner[k]);
                hi[k] = std::max(hi[k], corner[k]);
            }
        }
        mTriangles.push_back(tri);
    }
    if (mTriangles.empty()) {
        return;
    }
    // cells of the range, at most about 2 million
    mCell = std::max(range, 1e-3f);
    for (;;) {
        long cells = 1;
        for (int k = 0; k < 3; k++) {
            mN[k] = (int)std::ceil((hi[k] - mLo[k] + 2 * range) / mCell) + 1;
            cells *= mN[k];
        }
        if (cells <= 2000000) {
            break;
        }
        mCell *= 1.5f;
    }
    for (float& c : mLo) {
        c -= range;
    }
    // triangles by cells and by columns, counted, then filled
    auto cellRange = [&](const Triangle& tri, int i0[3], int i1[3]) {
        for (int k = 0; k < 3; k++) {
            const float lo = std::min({tri.a[k], tri.b[k], tri.c[k]}) - range;
            const float hiK = std::max({tri.a[k], tri.b[k], tri.c[k]}) + range;
            i0[k] = std::clamp((int)std::floor((lo - mLo[k]) / mCell), 0, mN[k] - 1);
            i1[k] = std::clamp((int)std::floor((hiK - mLo[k]) / mCell), 0, mN[k] - 1);
        }
    };
    auto columnRange = [&](const Triangle& tri, int i0[2], int i1[2]) {
        for (int k = 0; k < 2; k++) {
            const float lo = std::min({tri.a[k], tri.b[k], tri.c[k]});
            const float hiK = std::max({tri.a[k], tri.b[k], tri.c[k]});
            i0[k] = std::clamp((int)std::floor((lo - mLo[k]) / mCell), 0, mN[k] - 1);
            i1[k] = std::clamp((int)std::floor((hiK - mLo[k]) / mCell), 0, mN[k] - 1);
        }
    };
    mCellStart.assign((size_t)mN[0] * mN[1] * mN[2] + 1, 0);
    mColumnStart.assign((size_t)mN[0] * mN[1] + 1, 0);
    for (int pass = 0; pass < 2; pass++) {
        std::vector<uint32_t> cellFill, columnFill;
        if (pass == 1) {
            for (size_t i = 1; i < mCellStart.size(); i++) {
                mCellStart[i] += mCellStart[i - 1];
            }
            for (size_t i = 1; i < mColumnStart.size(); i++) {
                mColumnStart[i] += mColumnStart[i - 1];
            }
            mCellTriangles.resize(mCellStart.back());
            mColumnTriangles.resize(mColumnStart.back());
            cellFill.assign(mCellStart.begin(), mCellStart.end() - 1);
            columnFill.assign(mColumnStart.begin(), mColumnStart.end() - 1);
        }
        for (uint32_t t = 0; t < mTriangles.size(); t++) {
            int i0[3], i1[3];
            cellRange(mTriangles[t], i0, i1);
            for (int z = i0[2]; z <= i1[2]; z++) {
                for (int y = i0[1]; y <= i1[1]; y++) {
                    for (int x = i0[0]; x <= i1[0]; x++) {
                        const int cell = CellIndex(x, y, z);
                        if (pass == 0) {
                            mCellStart[cell + 1]++;
                        }
                        else {
                            mCellTriangles[cellFill[cell]++] = t;
                        }
                    }
                }
            }
            int c0[2], c1[2];
            columnRange(mTriangles[t], c0, c1);
            for (int y = c0[1]; y <= c1[1]; y++) {
                for (int x = c0[0]; x <= c1[0]; x++) {
                    const int column = y * mN[0] + x;
                    if (pass == 0) {
                        mColumnStart[column + 1]++;
                    }
                    else {
                        mColumnTriangles[columnFill[column]++] = t;
                    }
                }
            }
        }
    }
}

float ModelDistance::SignedDistance(const float p[3]) const
{
    if (mTriangles.empty()) {
        return mRange;
    }
    int c[3];
    bool outsideGrid = false;
    for (int k = 0; k < 3; k++) {
        c[k] = (int)std::floor((p[k] - mLo[k]) / mCell);
        outsideGrid = outsideGrid || c[k] < 0 || c[k] >= mN[k];
    }
    if (outsideGrid) {
        return mRange;
    }
    // the cell holds every triangle within the range of any of its points
    const float range2 = mRange * mRange;
    float best = range2;
    uint32_t bestTriangle = 0;
    bool found = false, bestInterior = false;
    const int cell = CellIndex(c[0], c[1], c[2]);
    for (uint32_t i = mCellStart[cell]; i < mCellStart[cell + 1]; i++) {
        const Triangle& tri = mTriangles[mCellTriangles[i]];
        bool interior;
        const float d2 = TriangleDistance2(p, tri.a, tri.b, tri.c, interior);
        if (d2 < best) {
            best = d2;
            bestTriangle = mCellTriangles[i];
            bestInterior = interior;
            found = true;
        }
    }
    if (!found) {
        return Inside(p) ? -mRange : mRange;
    }
    const float distance = std::sqrt(best);
    // the side of the surface: the normal of the nearest triangle, or at its edges and corners the
    // mean of the normals of the triangles as near (an approximation of the pseudo normal)
    float n[3];
    const Triangle& nearest = mTriangles[bestTriangle];
    std::copy(nearest.n, nearest.n + 3, n);
    if (!bestInterior) {
        const float tolerance = std::max(best * 1.0001f, best + 1e-12f);
        for (uint32_t i = mCellStart[cell]; i < mCellStart[cell + 1]; i++) {
            if (mCellTriangles[i] == bestTriangle) {
                continue;
            }
            const Triangle& tri = mTriangles[mCellTriangles[i]];
            bool interior;
            if (TriangleDistance2(p, tri.a, tri.b, tri.c, interior) <= tolerance) {
                for (int k = 0; k < 3; k++) {
                    n[k] += tri.n[k];
                }
            }
        }
    }
    const float side = (p[0] - nearest.a[0]) * n[0] + (p[1] - nearest.a[1]) * n[1]
        + (p[2] - nearest.a[2]) * n[2];
    if (distance < 1e-6f) {
        return 0;
    }
    return side >= 0 ? distance : -distance;
}

// Inside the model: the parity of its crossings along +Z, a little beside the point so the ray
// doesn't pass along an edge of the mesh, like the diagonal of a face
bool ModelDistance::Inside(const float point[3]) const
{
    const float p[3]
        = {point[0] + 0.6180340f * 1e-4f * mCell, point[1] + 0.3819660f * 1e-4f * mCell, point[2]};
    const int x = (int)std::floor((p[0] - mLo[0]) / mCell);
    const int y = (int)std::floor((p[1] - mLo[1]) / mCell);
    if (x < 0 || y < 0 || x >= mN[0] || y >= mN[1]) {
        return false;
    }
    const int column = y * mN[0] + x;
    int crossings = 0;
    for (uint32_t i = mColumnStart[column]; i < mColumnStart[column + 1]; i++) {
        const Triangle& tri = mTriangles[mColumnTriangles[i]];
        auto edge = [&](const float* u, const float* v) {
            return (v[0] - u[0]) * (p[1] - u[1]) - (v[1] - u[1]) * (p[0] - u[0]);
        };
        const float e0 = edge(tri.a, tri.b), e1 = edge(tri.b, tri.c), e2 = edge(tri.c, tri.a);
        if (!(e0 > 0 && e1 > 0 && e2 > 0) && !(e0 < 0 && e1 < 0 && e2 < 0)) {
            continue;
        }
        const float z = (e1 * tri.a[2] + e2 * tri.b[2] + e0 * tri.c[2]) / (e0 + e1 + e2);
        if (z > p[2]) {
            crossings++;
        }
    }
    return (crossings & 1) != 0;
}

}  // namespace CAMSimulator::Material
