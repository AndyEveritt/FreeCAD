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

#include "DexelSimulation.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <unordered_map>

namespace CAMSimulator
{

namespace
{

using Clock = std::chrono::steady_clock;

// Work (lines) between two snapshots of the background simulation, and the least between checks
// of the time budget in Update
constexpr double CheckpointWork = 5e5;
constexpr double UpdateChunkWork = 1e5;
// Memory of snapshot bricks after which every other snapshot is dropped
constexpr long CheckpointMemory = 512L * 1024 * 1024;
// Leaves of the layout at most, the finest level is dropped until it fits
constexpr int MaxLeaves = 400000;
// Levels finer than the cells of a tool for the grooves between its passes, at most
constexpr int MaxGrooveLevels = 2;
// Grooves get finer cells where they are along a path of this many tool radii at least
constexpr float MinGrooveLength = 4;

// the background simulation runs with low priority, rendering and the visible simulation
// preempt it
int WorkerThreads()
{
    return std::max(1, (int)std::thread::hardware_concurrency());
}

// A segment, or a piece of an arc, for building the layout
struct LayoutPiece
{
    int segment;
    float p0[3], p1[3];  // tool tip
    float a0[3], a1[3];  // tool axis
    float tolerance;     // largest deviation of the chord of an arc
    bool vertical;
};

std::vector<LayoutPiece> MakeLayoutPieces(
    const std::vector<CutSegment>& segments,
    const std::vector<std::unique_ptr<CutterProfile>>& tools
)
{
    std::vector<LayoutPiece> pieces;
    for (int s = 0; s < (int)segments.size(); s++) {
        const CutSegment& seg = segments[s];
        // pieces of arcs, the chords deviate at most a quarter of the tool radius
        const float tolerance = std::max(tools[seg.tool]->Radius() / 4, 1e-3f);
        int count = 1;
        if (seg.arc && seg.radius > tolerance) {
            const float maxAngle = 2 * std::acos(1 - tolerance / seg.radius);
            count = std::max(1, (int)std::ceil(std::abs(seg.sweep) / maxAngle));
        }
        for (int i = 0; i < count; i++) {
            LayoutPiece piece {};
            piece.segment = s;
            const float t0 = (float)i / count, t1 = (float)(i + 1) / count;
            seg.PositionAt(t0, piece.p0);
            seg.PositionAt(t1, piece.p1);
            seg.AxisAt(t0, piece.a0);
            seg.AxisAt(t1, piece.a1);
            piece.tolerance = seg.arc ? tolerance : 0;
            piece.vertical = piece.a0[2] > 0.999999f && piece.a1[2] > 0.999999f;
            pieces.push_back(piece);
        }
    }
    return pieces;
}

// Squared distance in XY from a point to the segment a-b
float DistanceXY2(float x, float y, const float a[3], const float b[3])
{
    const float dx = b[0] - a[0], dy = b[1] - a[1];
    const float len2 = dx * dx + dy * dy;
    const float t = len2 > 0 ? std::clamp(((x - a[0]) * dx + (y - a[1]) * dy) / len2, 0.0f, 1.0f) : 0.0f;
    const float ex = x - a[0] - t * dx, ey = y - a[1] - t * dy;
    return ex * ex + ey * ey;
}

// Grooves between a piece and the passes of the same tool beside it
struct PassGroove
{
    float spacing = 0;  // 0: no grooves
    // where the tool touches the surface the passes make, from the tool tip; both sides of the
    // tool where its axis lies in the surface
    float touch[2][3] = {};
    int sides = 0;
};


// Distance from a point to the segment a-b, squared, and the parameter of the closest point
float Distance2(const float p[3], const float a[3], const float b[3], float& param)
{
    float ab[3], ap[3], len2 = 0, dot = 0;
    for (int k = 0; k < 3; k++) {
        ab[k] = b[k] - a[k];
        ap[k] = p[k] - a[k];
        len2 += ab[k] * ab[k];
        dot += ab[k] * ap[k];
    }
    param = len2 > 0 ? std::clamp(dot / len2, 0.0f, 1.0f) : 0.0f;
    float d2 = 0;
    for (int k = 0; k < 3; k++) {
        const float d = ap[k] - param * ab[k];
        d2 += d * d;
    }
    return d2;
}

// Center of the lower end of a tool: the center of the ball of a ball end mill. Passes of a ball
// end mill leave grooves of the distance of their centers, with any tool axis.
void ToolCenter(const CutterProfile& tool, const float tip[3], const float axis[3], float out[3])
{
    // the radius of the lower end, not of a wider shank
    const float h = std::min(tool.RadiusBelow(tool.Radius()), tool.Height());
    for (int k = 0; k < 3; k++) {
        out[k] = tip[k] + axis[k] * h;
    }
}

// Run f(worker, i) for every i in [0, n) on the threads of the pool
template<class F>
void ParallelFor(ThreadPool& pool, int n, F f)
{
    std::atomic<int> next {0};
    pool.Run([&](int worker) {
        while (true) {
            const int first = next.fetch_add(64);
            if (first >= n) {
                break;
            }
            for (int i = first; i < std::min(n, first + 64); i++) {
                f(worker, i);
            }
        }
    });
}

// The grooves of the passes of the tools, for every piece: the distance to the passes of the same
// tool running beside it, like the stepover of a finishing operation, with any tool axis. None
// where there are no passes within the largest spacing of the tool, or where the tool doesn't
// leave visible grooves between such passes, like a flat end mill on a floor.
std::vector<PassGroove> PassGrooves(
    const std::vector<LayoutPiece>& pieces,
    const std::vector<CutSegment>& segments,
    const std::vector<std::unique_ptr<CutterProfile>>& tools,
    const std::vector<float>& maxSpacing,
    ThreadPool& pool
)
{
    // Grooves are visible where the surfaces of two passes meet at the ridge between them at more
    // than this angle (10 degrees)
    constexpr float GrooveAngle = 0.1745f;
    // Edges of a tool are computed for its axis rounded to this fraction, over this many tool
    // radii to either side, in this many bins
    constexpr int EdgeAxisSteps = 24;
    constexpr float EdgeExtent = 4;
    constexpr int EdgeBins = 257;
    // Samples along a piece at most
    constexpr int MaxSamples = 8;
    // Directions of passes beside each other differ by less than this (cosine)
    constexpr float ParallelCos = 0.9f;

    const int count = (int)pieces.size();
    std::vector<PassGroove> grooves(count);
    // tool centers at the ends of the pieces
    std::vector<std::array<float, 6>> centers(count);
    for (int i = 0; i < count; i++) {
        const CutterProfile& tool = *tools[segments[pieces[i].segment].tool];
        ToolCenter(tool, pieces[i].p0, pieces[i].a0, &centers[i][0]);
        ToolCenter(tool, pieces[i].p1, pieces[i].a1, &centers[i][3]);
    }
    // the pass beside a piece: its spacing, the frame of the passes (across to the next pass, up
    // from the surface they make towards the tool) and the tool axis in that frame
    struct Beside
    {
        float spacing = 0;
        float across[3], up[3];
        std::array<int, 3> key;
        float axisUp;
    };
    std::vector<Beside> beside(count);
    for (int t = 0; t < (int)tools.size(); t++) {
        const float range = maxSpacing[t];
        if (range <= 0) {
            continue;
        }
        // the pieces of the tool in a grid of cells of the largest spacing, in the cells they
        // pass through
        const float cell = range;
        std::unordered_map<int64_t, std::vector<int>> grid;
        auto key = [](int x, int y, int z) {
            return ((int64_t)(x & 0x1fffff) << 42) | ((int64_t)(y & 0x1fffff) << 21) | (int64_t)(z & 0x1fffff);
        };
        auto index = [&](float v) {
            return (int)std::floor(v / cell);
        };
        std::vector<int> own;
        for (int i = 0; i < count; i++) {
            if (segments[pieces[i].segment].tool != t) {
                continue;
            }
            own.push_back(i);
            const float* c0 = &centers[i][0];
            const float* c1 = &centers[i][3];
            const float through = 0.75f * cell * cell;  // half diagonal, squared
            for (int z = index(std::min(c0[2], c1[2])); z <= index(std::max(c0[2], c1[2])); z++) {
                for (int y = index(std::min(c0[1], c1[1])); y <= index(std::max(c0[1], c1[1])); y++) {
                    for (int x = index(std::min(c0[0], c1[0])); x <= index(std::max(c0[0], c1[0])); x++) {
                        const float mid[3] = {(x + 0.5f) * cell, (y + 0.5f) * cell, (z + 0.5f) * cell};
                        float param;
                        if (Distance2(mid, c0, c1, param) <= through) {
                            grid[key(x, y, z)].push_back(i);
                        }
                    }
                }
            }
        }
        struct Found
        {
            float distance;
            float offset[3];  // to the closest point of the other pass
        };
        std::vector<std::vector<Found>> founds(pool.Size());
        std::vector<std::vector<int>> nears(pool.Size());
        ParallelFor(pool, (int)own.size(), [&](int worker, int o) {
            const int i = own[o];
            std::vector<Found>& found = founds[worker];
            std::vector<int>& near = nears[worker];
            const float* c0 = &centers[i][0];
            const float* c1 = &centers[i][3];
            float dir[3];
            float len = 0;
            for (int k = 0; k < 3; k++) {
                dir[k] = c1[k] - c0[k];
                len += dir[k] * dir[k];
            }
            len = std::sqrt(len);
            if (len <= 1e-6f) {
                return;
            }
            for (float& d : dir) {
                d /= len;
            }
            const int samples = std::clamp((int)std::ceil(len / range), 1, MaxSamples);
            found.clear();
            for (int k = 0; k < samples; k++) {
                const float s = (k + 0.5f) / samples;
                float pt[3];
                for (int m = 0; m < 3; m++) {
                    pt[m] = c0[m] + s * (c1[m] - c0[m]);
                }
                // the pieces in the cells around the sample, once
                near.clear();
                const int gx = index(pt[0]), gy = index(pt[1]), gz = index(pt[2]);
                for (int z = gz - 1; z <= gz + 1; z++) {
                    for (int y = gy - 1; y <= gy + 1; y++) {
                        for (int x = gx - 1; x <= gx + 1; x++) {
                            const auto it = grid.find(key(x, y, z));
                            if (it != grid.end()) {
                                near.insert(near.end(), it->second.begin(), it->second.end());
                            }
                        }
                    }
                }
                std::sort(near.begin(), near.end());
                near.erase(std::unique(near.begin(), near.end()), near.end());
                Found best {range, {0, 0, 0}};
                float best2 = range * range;
                for (const int j : near) {
                    // not the same pass: the neighboring moves along it are excluded
                    if (std::abs(pieces[j].segment - pieces[i].segment) <= 1) {
                        continue;
                    }
                    const float* o0 = &centers[j][0];
                    const float* o1 = &centers[j][3];
                    float along = 0, olen2 = 0;
                    for (int m = 0; m < 3; m++) {
                        along += (o1[m] - o0[m]) * dir[m];
                        olen2 += (o1[m] - o0[m]) * (o1[m] - o0[m]);
                    }
                    if (along * along < ParallelCos * ParallelCos * olen2) {
                        continue;
                    }
                    // beside the sample, not ahead of it along the same line
                    float param;
                    const float d2 = Distance2(pt, o0, o1, param);
                    if (param <= 0 || param >= 1 || d2 <= 0.01f * range * range || d2 >= best2) {
                        continue;
                    }
                    best2 = d2;
                    best.distance = std::sqrt(d2);
                    for (int m = 0; m < 3; m++) {
                        best.offset[m] = o0[m] + param * (o1[m] - o0[m]) - pt[m];
                    }
                }
                if (best.distance < range) {
                    found.push_back(best);
                }
            }
            // passes beside most of the piece
            if (found.empty() || found.size() * 2 < (size_t)samples) {
                return;
            }
            std::nth_element(found.begin(), found.begin() + found.size() / 2, found.end(), [](const Found& a, const Found& b) {
                return a.distance < b.distance;
            });
            const Found& median = found[found.size() / 2];
            // the frame of the passes: along the pass, across to the next pass, up from the
            // surface they make towards the tool
            Beside& b = beside[i];
            const float* off = median.offset;
            const float od = off[0] * dir[0] + off[1] * dir[1] + off[2] * dir[2];
            float spacing = 0;
            for (int m = 0; m < 3; m++) {
                b.across[m] = off[m] - od * dir[m];
                spacing += b.across[m] * b.across[m];
            }
            spacing = std::sqrt(spacing);
            if (spacing <= 0) {
                return;
            }
            for (float& c : b.across) {
                c /= spacing;
            }
            float* up = b.up;
            up[0] = dir[1] * b.across[2] - dir[2] * b.across[1];
            up[1] = dir[2] * b.across[0] - dir[0] * b.across[2];
            up[2] = dir[0] * b.across[1] - dir[1] * b.across[0];
            const float* axis = pieces[i].a0;
            float axisUp = up[0] * axis[0] + up[1] * axis[1] + up[2] * axis[2];
            if (axisUp < 0) {
                for (int m = 0; m < 3; m++) {
                    up[m] = -up[m];
                }
                axisUp = -axisUp;
            }
            const float axisAcross = b.across[0] * axis[0] + b.across[1] * axis[1] + b.across[2] * axis[2];
            const float axisAlong = dir[0] * axis[0] + dir[1] * axis[1] + dir[2] * axis[2];
            // the edge of the tool at this axis, the same for many pieces
            const auto quantize = [](float x) {
                return (int)std::lround(x * EdgeAxisSteps);
            };
            b.key = {quantize(std::abs(axisAlong)), quantize(axisAcross), quantize(axisUp)};
            b.axisUp = axisUp;
            b.spacing = spacing;
        });
    }

    // the edges of the tools at the axes of the pieces, computed once
    std::map<std::pair<int, std::array<int, 3>>, int> edgeIndex;
    std::vector<std::pair<int, std::array<int, 3>>> edgeKeys;
    for (int i = 0; i < count; i++) {
        if (beside[i].spacing > 0) {
            const auto k = std::make_pair(segments[pieces[i].segment].tool, beside[i].key);
            if (edgeIndex.emplace(k, (int)edgeKeys.size()).second) {
                edgeKeys.push_back(k);
            }
        }
    }
    std::vector<CutterProfile::SweptEdge> edges(edgeKeys.size());
    ParallelFor(pool, (int)edgeKeys.size(), [&](int, int e) {
        const CutterProfile& tool = *tools[edgeKeys[e].first];
        const std::array<int, 3>& key = edgeKeys[e].second;
        float frameAxis[3] = {
            key[0] / (float)EdgeAxisSteps,
            key[1] / (float)EdgeAxisSteps,
            key[2] / (float)EdgeAxisSteps,
        };
        const float len = std::sqrt(frameAxis[0] * frameAxis[0] + frameAxis[1] * frameAxis[1] + frameAxis[2] * frameAxis[2]);
        for (float& c : frameAxis) {
            c /= len;
        }
        edges[e] = tool.SweptLowerEdge(frameAxis, EdgeExtent * tool.Radius(), EdgeBins);
    });

    // the grooves the edges leave between passes at their spacing
    ParallelFor(pool, count, [&](int, int i) {
        const Beside& b = beside[i];
        if (b.spacing <= 0) {
            return;
        }
        const auto& edge = edges[edgeIndex.at({segments[pieces[i].segment].tool, b.key})];
        float angle, touchAcross, touchUp;
        edge.Groove(b.spacing, angle, touchAcross, touchUp);
        if (angle < GrooveAngle) {
            return;
        }
        PassGroove& groove = grooves[i];
        groove.spacing = b.spacing;
        // with the axis in the surface, the tool may touch it on either side
        groove.sides = b.axisUp < 0.1f ? 2 : 1;
        for (int side = 0; side < groove.sides; side++) {
            const float sign = side == 0 ? 1.0f : -1.0f;
            for (int m = 0; m < 3; m++) {
                groove.touch[side][m] = touchAcross * b.across[m] + sign * touchUp * b.up[m];
            }
        }
    });
    return grooves;
}

// The pieces with grooves that can make the final surface. The lower surface of the vertical tools
// is followed on a grid in XY, the lowest of all their moves. A piece is above the final surface
// where moves cut lower by more than 'margin' everywhere along it, like the passes of a step down
// above the last one: its grooves are cut away. Tilted tools are not followed, their pieces are
// kept, and other moves can't remove them.
std::vector<uint8_t> OnFinalSurface(
    const std::vector<LayoutPiece>& pieces,
    const std::vector<CutSegment>& segments,
    const std::vector<std::unique_ptr<CutterProfile>>& tools,
    const std::vector<PassGroove>& grooves,
    const float lo[3],
    const float hi[3],
    float cell,
    const std::vector<float>& margin,
    ThreadPool& pool
)
{
    const int count = (int)pieces.size();
    std::vector<uint8_t> final(count, 1);
    // at most this many cells
    constexpr float MaxCells = 4e6f;
    cell = std::max(cell, std::sqrt((hi[0] - lo[0]) * (hi[1] - lo[1]) / MaxCells));
    const int nx = std::max(1, (int)std::ceil((hi[0] - lo[0]) / cell));
    const int ny = std::max(1, (int)std::ceil((hi[1] - lo[1]) / cell));
    std::vector<std::atomic<float>> lowest((size_t)nx * ny);
    for (auto& z : lowest) {
        z.store(std::numeric_limits<float>::infinity(), std::memory_order_relaxed);
    }
    auto follows = [&](const LayoutPiece& p) {
        return p.vertical && tools[segments[p.segment].tool]->HasSingleAxialIntervals();
    };
    ParallelFor(pool, count, [&](int, int i) {
        const LayoutPiece& p = pieces[i];
        if (!follows(p)) {
            return;
        }
        const CutterProfile& tool = *tools[segments[p.segment].tool];
        const float r = tool.Radius();
        const int x0 = std::max(0, (int)std::floor((std::min(p.p0[0], p.p1[0]) - r - lo[0]) / cell));
        const int x1 = std::min(nx - 1, (int)std::floor((std::max(p.p0[0], p.p1[0]) + r - lo[0]) / cell));
        const int y0 = std::max(0, (int)std::floor((std::min(p.p0[1], p.p1[1]) - r - lo[1]) / cell));
        const int y1 = std::min(ny - 1, (int)std::floor((std::max(p.p0[1], p.p1[1]) + r - lo[1]) / cell));
        const float dx = p.p1[0] - p.p0[0], dy = p.p1[1] - p.p0[1];
        const float len2 = dx * dx + dy * dy;
        for (int y = y0; y <= y1; y++) {
            for (int x = x0; x <= x1; x++) {
                const float cx = lo[0] + (x + 0.5f) * cell, cy = lo[1] + (y + 0.5f) * cell;
                // at the closest approach, at least as high as the tool goes there
                const float t = len2 > 0 ? std::clamp(((cx - p.p0[0]) * dx + (cy - p.p0[1]) * dy) / len2, 0.0f, 1.0f)
                                         : 0.0f;
                const float ex = cx - p.p0[0] - t * dx, ey = cy - p.p0[1] - t * dy;
                const float d = std::sqrt(ex * ex + ey * ey);
                const Interval* iv;
                if (d > r || tool.AxialIntervals(d, iv) == 0) {
                    continue;
                }
                const float z = p.p0[2] + t * (p.p1[2] - p.p0[2]) + iv[0].lo;
                std::atomic<float>& low = lowest[(size_t)y * nx + x];
                float current = low.load(std::memory_order_relaxed);
                while (z < current && !low.compare_exchange_weak(current, z, std::memory_order_relaxed)) {
                }
            }
        }
    });
    ParallelFor(pool, count, [&](int, int i) {
        const LayoutPiece& p = pieces[i];
        if (grooves[i].spacing <= 0 || !follows(p)) {
            return;
        }
        const int t = segments[p.segment].tool;
        const Interval* iv;
        if (tools[t]->AxialIntervals(0, iv) == 0) {
            return;
        }
        const float length = std::hypot(p.p1[0] - p.p0[0], p.p1[1] - p.p0[1]);
        const int samples = std::max(1, (int)std::ceil(length / cell)) + 1;
        for (int k = 0; k < samples; k++) {
            const float s = samples > 1 ? (float)k / (samples - 1) : 0.0f;
            const float px = p.p0[0] + s * (p.p1[0] - p.p0[0]), py = p.p0[1] + s * (p.p1[1] - p.p0[1]);
            const int x = std::clamp((int)std::floor((px - lo[0]) / cell), 0, nx - 1);
            const int y = std::clamp((int)std::floor((py - lo[1]) / cell), 0, ny - 1);
            const float z = p.p0[2] + s * (p.p1[2] - p.p0[2]) + iv[0].lo;
            if (z <= lowest[(size_t)y * nx + x].load(std::memory_order_relaxed) + margin[t]) {
                return;
            }
        }
        final[i] = 0;
    });
    return final;
}

// The highest material of the stock that a piece can cut, for every piece: the stock is followed on
// a grid in XY with the top of its material, lowered where vertical tools have removed everything
// above their lower surface. Pieces of tilted tools can cut anywhere up to the top of the stock.
std::vector<float> MaterialTops(
    const std::vector<LayoutPiece>& pieces,
    const std::vector<CutSegment>& segments,
    const std::vector<std::unique_ptr<CutterProfile>>& tools,
    const float lo[3],
    const float hi[3]
)
{
    std::vector<float> tops(pieces.size(), hi[2]);
    float smallest = 1e30f;
    for (const auto& tool : tools) {
        if (tool->Radius() > 0) {
            smallest = std::min(smallest, tool->Radius());
        }
    }
    if (smallest >= 1e30f) {
        return tops;
    }
    const float longest = std::max(hi[0] - lo[0], hi[1] - lo[1]);
    const float cell = std::max(smallest / 2, longest / 1024);
    const int nx = std::max(1, (int)std::ceil((hi[0] - lo[0]) / cell));
    const int ny = std::max(1, (int)std::ceil((hi[1] - lo[1]) / cell));
    std::vector<float> top((size_t)nx * ny, hi[2]);

    for (size_t i = 0; i < pieces.size(); i++) {
        const LayoutPiece& p = pieces[i];
        if (!p.vertical) {
            continue;
        }
        const CutterProfile& tool = *tools[segments[p.segment].tool];
        const float r = tool.Radius() + p.tolerance;
        const int x0 = std::max(0, (int)std::floor((std::min(p.p0[0], p.p1[0]) - r - lo[0]) / cell));
        const int x1 = std::min(nx - 1, (int)std::floor((std::max(p.p0[0], p.p1[0]) + r - lo[0]) / cell));
        const int y0 = std::max(0, (int)std::floor((std::min(p.p0[1], p.p1[1]) - r - lo[1]) / cell));
        const int y1 = std::min(ny - 1, (int)std::floor((std::max(p.p0[1], p.p1[1]) + r - lo[1]) / cell));
        // the material under the tool before the move
        const float reach = r + cell * 0.7072f;
        float highest = -1e30f;
        bool any = false;
        for (int y = y0; y <= y1; y++) {
            for (int x = x0; x <= x1; x++) {
                const float cx = lo[0] + (x + 0.5f) * cell, cy = lo[1] + (y + 0.5f) * cell;
                if (DistanceXY2(cx, cy, p.p0, p.p1) <= reach * reach) {
                    highest = std::max(highest, top[(size_t)y * nx + x]);
                    any = true;
                }
            }
        }
        tops[i] = any ? highest : hi[2];

        // remove the material of the cells completely under the tool, where it reaches above the
        // material and leaves none above its lower surface
        if (!tool.IsRadiallyMonotonic() || !tool.HasSingleAxialIntervals()) {
            continue;
        }
        const float zTip = std::max(p.p0[2], p.p1[2]);
        for (int y = y0; y <= y1; y++) {
            for (int x = x0; x <= x1; x++) {
                float far = 0;
                for (int c = 0; c < 4; c++) {
                    const float cx = lo[0] + (x + (c & 1)) * cell, cy = lo[1] + (y + (c >> 1)) * cell;
                    far = std::max(far, DistanceXY2(cx, cy, p.p0, p.p1));
                }
                far = std::sqrt(far);
                if (far >= tool.Radius()) {
                    continue;
                }
                const Interval* iv;
                if (tool.AxialIntervals(far, iv) != 1 || iv[0].hi < tool.Height() - 1e-4f) {
                    continue;
                }
                float& h = top[(size_t)y * nx + x];
                if (zTip + tool.Height() >= h) {
                    h = std::min(h, zTip + iv[0].lo);
                }
            }
        }
    }
    return tops;
}
}  // namespace

float CutSegment::StepParam(int step) const
{
    return std::min(1.0f, (float)step / std::max(1, sweepSteps));
}

void CutSegment::PositionAt(float t, float pos[3]) const
{
    if (arc) {
        const float angle = startAngle + t * sweep;
        pos[0] = center[0] - std::sin(angle) * radius;
        pos[1] = center[1] + std::cos(angle) * radius;
        pos[2] = start[2] + t * (end[2] - start[2]);
        return;
    }
    for (int k = 0; k < 3; k++) {
        pos[k] = start[k] + t * (end[k] - start[k]);
    }
}

void CutSegment::AxisAt(float t, float axis[3]) const
{
    float len = 0;
    for (int k = 0; k < 3; k++) {
        axis[k] = axis0[k] + t * (axis1[k] - axis0[k]);
        len += axis[k] * axis[k];
    }
    len = std::sqrt(len);
    for (int k = 0; k < 3; k++) {
        axis[k] = len > 0 ? axis[k] / len : (k == 2 ? 1.0f : 0.0f);
    }
}

DexelSimulation::DexelSimulation()
    : mWorkerPool(WorkerThreads(), true)
{}

DexelSimulation::~DexelSimulation()
{
    Stop();
}

void DexelSimulation::Stop()
{
    mStopWorker = true;
    if (mWorker.joinable()) {
        mWorker.join();
    }
    mStopWorker = false;
}

void DexelSimulation::Start(
    std::vector<std::unique_ptr<CutterProfile>> tools,
    std::vector<CutSegment> segments,
    const std::vector<float>& xyz,
    const std::vector<uint32_t>& triangles,
    float quality
)
{
    Stop();
    mTools = std::move(tools);
    mSegments = std::move(segments);
    mCheckpoints.clear();

    BuildLayout(xyz, quality);
    mInitial.Init(mLayout);
    mInitial.SetMesh(xyz, triangles, mPool);
    mInitial.Freeze();
    mInitial.TakeDirtyLeaves();
    mStock.Init(mLayout);
    mStock.Restore(mInitial.Bricks());
    mStock.SetTopZ(mInitial.TopZ());

    // timeline and work estimate
    mBase.assign(1, 0);
    mWork.assign(1, 0);
    for (const CutSegment& seg : mSegments) {
        mBase.push_back(mBase.back() + seg.steps);
    }
    std::vector<double> work(mSegments.size(), 0);
    std::vector<std::vector<CutMove>> moves(mPool.Size());
    ParallelFor(mPool, (int)mSegments.size(), [&](int worker, int s) {
        moves[worker].clear();
        AppendMoves(mBase[s], mBase[s + 1], moves[worker]);
        for (const CutMove& move : moves[worker]) {
            work[s] += (double)mInitial.EstimateWork(move);
        }
    });
    for (const double w : work) {
        mWork.push_back(mWork.back() + w);
    }
    mCurrent = 0;
    mTarget = 0;

    mWorker = std::thread([this] { Worker(); });
}

void DexelSimulation::BuildLayout(const std::vector<float>& xyz, float quality)
{
    float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
    for (size_t i = 0; i + 2 < xyz.size(); i += 3) {
        for (int k = 0; k < 3; k++) {
            lo[k] = std::min(lo[k], xyz[i + k]);
            hi[k] = std::max(hi[k], xyz[i + k]);
        }
    }
    if (lo[0] > hi[0]) {
        for (int k = 0; k < 3; k++) {
            lo[k] = hi[k] = 0;
        }
    }
    const float longest = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], 1e-3f});
    const float topZ = hi[2];

    // Cells of the stock away from the tools: 32 to 104 along the longest side. Cells where a
    // tool cuts: 0.5 to 0.1 times the size of the features of the tool, and where it leaves
    // grooves between its passes, like a ball end mill, 1.25 to 0.25 times the stepover, so
    // the grooves are resolved.
    const float q = std::clamp(quality, 1.0f, 10.0f);
    const float baseCell = longest / (24 + 8 * q);
    const float relative = 0.5f * std::pow(0.2f, (q - 1) / 9);
    const float minCell = std::max(0.005f, longest / 4096);
    std::vector<float> toolCell;
    for (const auto& tool : mTools) {
        toolCell.push_back(std::clamp(relative * tool->FeatureSize(), minCell, baseCell));
    }

    const std::vector<LayoutPiece> pieces = MakeLayoutPieces(mSegments, mTools);
    // Grooves between passes are resolved with cells of this fraction of their spacing. Spacings
    // that don't need cells finer than those of the tool, or that can't be resolved at this
    // quality, are not looked for.
    const float grooveFraction = 2.5f * relative;
    std::vector<float> maxSpacing(mTools.size(), 0);
    for (size_t t = 0; t < mTools.size(); t++) {
        if (grooveFraction <= 0.5f) {
            maxSpacing[t] = std::min(2 * mTools[t]->Radius(), toolCell[t] / (grooveFraction * std::sqrt(2.0f)));
        }
    }
    const std::vector<PassGroove> grooves = PassGrooves(pieces, mSegments, mTools, maxSpacing, mPool);
    const std::vector<float> materialTop = MaterialTops(pieces, mSegments, mTools, lo, hi);

    // The levels of the layout are powers of two apart: cells are aligned to those of the tool
    // cutting the longest path, so it gets the cells asked for, the others the closest level.
    std::vector<double> toolLength(mTools.size(), 0);
    for (const LayoutPiece& piece : pieces) {
        toolLength[mSegments[piece.segment].tool] += std::sqrt(
            (piece.p1[0] - piece.p0[0]) * (piece.p1[0] - piece.p0[0]) + (piece.p1[1] - piece.p0[1]) * (piece.p1[1] - piece.p0[1])
            + (piece.p1[2] - piece.p0[2]) * (piece.p1[2] - piece.p0[2])
        );
    }
    const float reference = toolCell.empty()
        ? baseCell
        : toolCell[std::max_element(toolLength.begin(), toolLength.end()) - toolLength.begin()];
    auto align = [&](float cell) {
        float aligned = reference * std::exp2((float)std::lround(std::log2(cell / reference)));
        while (aligned < minCell) {
            aligned *= 2;
        }
        return aligned;
    };
    for (float& cell : toolCell) {
        cell = align(cell);
    }

    // grooves only where they can make the final surface, within the cells of the tool
    const std::vector<uint8_t> final = OnFinalSurface(
        pieces,
        mSegments,
        mTools,
        grooves,
        lo,
        hi,
        toolCell.empty() ? baseCell : *std::min_element(toolCell.begin(), toolCell.end()),
        toolCell,
        mPool
    );

    // the cells of the pieces: of the tool, and of the grooves between passes
    std::vector<float> grooveCell(pieces.size(), 0);
    std::map<std::pair<int, int>, double> grooveLength;
    std::vector<int> grooveFiner(pieces.size(), 0);
    for (size_t i = 0; i < pieces.size(); i++) {
        const int t = mSegments[pieces[i].segment].tool;
        if (grooves[i].spacing > 0 && final[i]) {
            // levels below the tool cells, the closest to the cells for the spacing, at most two:
            // finer grooves are too small to see
            const float wanted = grooveFraction * grooves[i].spacing;
            const int finer = std::clamp((int)std::lround(std::log2(toolCell[t] / wanted)), 0, MaxGrooveLevels);
            if (finer > 0) {
                grooveFiner[i] = finer;
                const LayoutPiece& piece = pieces[i];
                grooveLength[{t, finer}] += std::sqrt(
                    (piece.p1[0] - piece.p0[0]) * (piece.p1[0] - piece.p0[0])
                    + (piece.p1[1] - piece.p0[1]) * (piece.p1[1] - piece.p0[1])
                    + (piece.p1[2] - piece.p0[2]) * (piece.p1[2] - piece.p0[2])
                );
            }
        }
    }
    // grooves along a short path only are not worth a finer level
    float fineCell = baseCell;
    for (size_t i = 0; i < pieces.size(); i++) {
        const int t = mSegments[pieces[i].segment].tool;
        fineCell = std::min(fineCell, toolCell[t]);
        const int finer = grooveFiner[i];
        if (finer > 0 && grooveLength[{t, finer}] >= MinGrooveLength * mTools[t]->Radius()) {
            grooveCell[i] = std::max(minCell, toolCell[t] / (float)(1 << finer));
            fineCell = std::min(fineCell, grooveCell[i]);
        }
    }
    int levels = std::clamp((int)std::floor(std::log2(baseCell / fineCell) + 1e-3), 0, 12);

    while (true) {
        auto layout = std::make_shared<DexelLayout>();
        layout->Init(lo, hi, fineCell, levels);
        for (size_t i = 0; i < pieces.size(); i++) {
            const LayoutPiece& piece = pieces[i];
            const CutSegment& seg = mSegments[piece.segment];
            const CutterProfile& tool = *mTools[seg.tool];
            const int level = std::clamp(
                levels - (int)std::lround(std::log2(toolCell[seg.tool] / fineCell)),
                0,
                levels
            );
            const float* p0 = piece.p0;
            const float* p1 = piece.p1;
            const float* a0 = piece.a0;
            const float* a1 = piece.a1;
            LayoutRegion region {};
            const float axisZ = std::min(a0[2], a1[2]);
            float length = tool.Height();
            if (axisZ >= 0.05f) {
                const float side = tool.Radius() * std::sqrt(std::max(0.0f, 1 - axisZ * axisZ));
                length = std::clamp((topZ - std::min(p0[2], p1[2]) + side) / axisZ, 0.0f, length);
            }
            if (piece.vertical) {
                std::copy(p0, p0 + 3, region.a);
                std::copy(p1, p1 + 3, region.b);
                region.radius = tool.RadiusBelow(length) + piece.tolerance;
                region.vertical = true;
                region.zlo = std::min(p0[2], p1[2]);
                // up to the material left by the moves before, not to the top of the stock
                region.zhi = std::min(std::max(p0[2], p1[2]) + length, materialTop[i]);
            }
            else {
                // spheres around the middle of the cutting part of the tool
                float turn = 0;
                for (int k = 0; k < 3; k++) {
                    region.a[k] = p0[k] + a0[k] * length / 2;
                    region.b[k] = p1[k] + a1[k] * length / 2;
                    turn = std::max(turn, std::abs(a1[k] - a0[k]));
                }
                region.radius = std::hypot(tool.RadiusBelow(length), length / 2) + turn * turn * length / 4
                    + piece.tolerance;
                region.vertical = false;
            }
            region.UpdateBounds();
            if (region.lo[2] > topZ || region.zhi < region.zlo) {
                continue;  // above the stock, or in material removed before
            }
            if (level > 0) {
                layout->Refine(region, level);
            }
            if (grooveCell[i] > 0) {
                // the grooves: finer cells around the path of the point where the tool touches the
                // surface, half way to the passes beside it
                const int grooveLevel = std::clamp(
                    levels - (int)std::lround(std::log2(grooveCell[i] / fineCell)),
                    0,
                    levels
                );
                if (grooveLevel > level) {
                    const PassGroove& groove = grooves[i];
                    for (int side = 0; side < groove.sides; side++) {
                        LayoutRegion band {};
                        for (int k = 0; k < 3; k++) {
                            band.a[k] = p0[k] + groove.touch[side][k];
                            band.b[k] = p1[k] + groove.touch[side][k];
                        }
                        band.radius = 0.5f * groove.spacing + 0.5f * grooveCell[i] + piece.tolerance;
                        band.vertical = false;
                        band.UpdateBounds();
                        layout->Refine(band, grooveLevel);
                    }
                }
            }
        }
        layout->Finish(&mPool);
        if (layout->LeafCount() <= MaxLeaves || levels == 0) {
            mLayout = layout;
            break;
        }
        // too large, drop the finest level
        levels--;
        fineCell *= 2;
    }
    mArcTolerance = mLayout->FineCell() * 0.25f;
}

long DexelSimulation::SnapshotStep() const
{
    std::lock_guard<std::mutex> lock(mMutex);
    return mCheckpoints.empty() ? 0 : mCheckpoints.back().step;
}

void DexelSimulation::SetTarget(long step)
{
    mTarget = std::clamp(step, 0L, TotalSteps());
}

void DexelSimulation::AppendMoves(long from, long to, std::vector<CutMove>& moves) const
{
    if (to <= from || mSegments.empty()) {
        return;
    }
    const float tolerance = mArcTolerance;
    size_t s = std::upper_bound(mBase.begin(), mBase.end(), from) - mBase.begin() - 1;
    for (; s < mSegments.size() && mBase[s] < to; s++) {
        const CutSegment& seg = mSegments[s];
        const int k1 = (int)std::clamp(from - mBase[s], 0L, (long)seg.steps);
        const int k2 = (int)std::clamp(to - mBase[s], 0L, (long)seg.steps);
        if (k2 <= k1) {
            continue;
        }
        const float t1 = seg.StepParam(k1);
        const float t2 = seg.StepParam(k2);
        if (t2 <= t1 && k1 > 0) {
            continue;  // steps after the end of the motion
        }
        int pieces = 1;
        if (seg.arc && seg.radius > 0) {
            // chords deviating at most 'tolerance' from the arc
            const float maxAngle = tolerance < seg.radius ? 2 * std::acos(1 - tolerance / seg.radius)
                                                          : 1.5f;
            pieces = std::max(1, (int)std::ceil(std::abs(seg.sweep) * (t2 - t1) / maxAngle));
        }
        for (int i = 0; i < pieces; i++) {
            const float ta = t1 + (t2 - t1) * i / pieces;
            const float tb = t1 + (t2 - t1) * (i + 1) / pieces;
            CutMove move;
            move.tool = mTools[seg.tool].get();
            move.operationTag = OperationTag(seg.operation);
            seg.PositionAt(ta, move.p0);
            seg.PositionAt(tb, move.p1);
            seg.AxisAt(ta, move.a0);
            seg.AxisAt(tb, move.a1);
            moves.push_back(move);
        }
    }
}

double DexelSimulation::WorkAt(long step) const
{
    if (step <= 0 || mSegments.empty()) {
        return 0;
    }
    if (step >= TotalSteps()) {
        return mWork.back();
    }
    const size_t s = std::upper_bound(mBase.begin(), mBase.end(), step) - mBase.begin() - 1;
    const double fraction = (double)(step - mBase[s]) / mSegments[s].steps;
    return mWork[s] + fraction * (mWork[s + 1] - mWork[s]);
}

long DexelSimulation::AdvanceByWork(long from, long work) const
{
    const double target = WorkAt(from) + work;
    if (target >= mWork.back()) {
        return TotalSteps();
    }
    // first segment ending after the target work
    const size_t s = std::upper_bound(mWork.begin(), mWork.end(), target) - mWork.begin() - 1;
    const double segmentWork = mWork[s + 1] - mWork[s];
    long step = mBase[s];
    if (segmentWork > 0) {
        step += (long)std::ceil((target - mWork[s]) / segmentWork * mSegments[s].steps);
    }
    return std::clamp(step, from + 1, TotalSteps());
}

void DexelSimulation::Worker()
{
    // the snapshots are a convenience, rendering and the visible simulation come first
    ThreadPool::LowerCurrentThreadPriority();
    DexelStock stock;
    stock.Init(mLayout);
    stock.Restore(mInitial.Bricks());
    stock.SetTopZ(mInitial.TopZ());
    double checkpointWork = CheckpointWork;
    long memory = 0;
    long step = 0;
    std::vector<CutMove> moves;
    while (step < TotalSteps() && !mStopWorker) {
        const long next = AdvanceByWork(step, (long)checkpointWork);
        moves.clear();
        AppendMoves(step, next, moves);
        stock.Cut(moves, mWorkerPool, &mStopWorker);
        if (mStopWorker) {
            break;
        }
        step = next;
        stock.Freeze();
        // the copied bricks and the list of bricks of the snapshot
        memory += stock.TakeNewBrickBytes() + (long)(stock.Bricks().size() * sizeof(DexelStock::BrickRef));

        std::lock_guard<std::mutex> lock(mMutex);
        mCheckpoints.push_back({step, stock.Bricks()});
        if (memory > CheckpointMemory && mCheckpoints.size() > 2) {
            // keep every other snapshot, and the latest
            std::vector<Checkpoint> kept;
            for (size_t i = 0; i < mCheckpoints.size(); i++) {
                if (i % 2 == 1 || i + 1 == mCheckpoints.size()) {
                    kept.push_back(std::move(mCheckpoints[i]));
                }
            }
            mCheckpoints = std::move(kept);
            memory /= 2;
            checkpointWork *= 2;
        }
    }
}

bool DexelSimulation::Update(double budgetMs)
{
    if (mCurrent == mTarget) {
        return false;
    }
    const auto deadline = Clock::now() + std::chrono::duration<double, std::milli>(budgetMs);
    bool changed = false;

    // the closest snapshot before the target
    long bestStep = -1;
    DexelStock::Snapshot best;
    {
        std::lock_guard<std::mutex> lock(mMutex);
        auto it = std::upper_bound(
            mCheckpoints.begin(),
            mCheckpoints.end(),
            mTarget,
            [](long step, const Checkpoint& c) { return step < c.step; }
        );
        if (it != mCheckpoints.begin()) {
            --it;
            bestStep = it->step;
            best = it->bricks;
        }
    }
    if (mTarget < mCurrent || bestStep > mCurrent) {
        if (bestStep >= 0 && bestStep <= mTarget) {
            mStock.Restore(best);
            mCurrent = bestStep;
        }
        else {
            mStock.Restore(mInitial.Bricks());
            mCurrent = 0;
        }
        changed = true;
    }

    // chunks of the work that fits in half of the remaining time, at the measured speed: larger
    // chunks keep more threads busy
    std::vector<CutMove> moves;
    while (mCurrent < mTarget) {
        const auto start = Clock::now();
        const double remaining = std::chrono::duration<double, std::milli>(deadline - start).count();
        const double chunk = std::clamp(mWorkPerMs * remaining / 2, UpdateChunkWork, 4 * CheckpointWork);
        const long next = std::min(mTarget, AdvanceByWork(mCurrent, (long)chunk));
        moves.clear();
        AppendMoves(mCurrent, next, moves);
        mStock.Cut(moves, mPool);
        const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        if (ms > 0.5) {
            mWorkPerMs = 0.7 * mWorkPerMs + 0.3 * (WorkAt(next) - WorkAt(mCurrent)) / ms;
        }
        mCurrent = next;
        changed = true;
        if (Clock::now() > deadline) {
            break;
        }
    }
    return changed;
}

}  // namespace CAMSimulator
