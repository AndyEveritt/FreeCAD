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

#include "MaterialSimulation.h"

#include "ThreadPool.h"

#include <algorithm>
#include <unordered_map>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace CAMSimulator::Material
{

namespace
{

using Clock = std::chrono::steady_clock;

double MsSince(Clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

// Time of cutting between checkpoints: going to any position cuts at most about this long
constexpr double CheckpointMs = 40;
// Time of a batch of the frontier: a new target waits for it at most
constexpr double BatchMs = 15;
// While the target is ahead of the frontier, its progress is shown this often: meshing and drawing
// every state would take the time from the simulation
constexpr double ProgressMs = 400;
// Memory of the bricks kept for checkpoints, beyond it every other checkpoint is dropped
constexpr long CheckpointBudget = 768L << 20;

// Material top along one of the six axis directions, on a grid across it: an upper bound of
// (point . direction) over the material in each cell
struct HeightMap
{
    int axis = 2;
    double sign = 1;
    int u = 0, v = 1;
    double lo[2] = {0, 0};
    double cell = 1;
    int n[2] = {1, 1};
    std::vector<float> top;

    void Init(int a, double s, const double stockLo[3], const double stockHi[3], double c)
    {
        axis = a;
        sign = s;
        u = (a + 1) % 3;
        v = (a + 2) % 3;
        cell = c;
        lo[0] = stockLo[u];
        lo[1] = stockLo[v];
        n[0] = std::max(1, (int)std::ceil((stockHi[u] - stockLo[u]) / c));
        n[1] = std::max(1, (int)std::ceil((stockHi[v] - stockLo[v]) / c));
        top.assign((size_t)n[0] * n[1], (float)(s > 0 ? stockHi[a] : -stockLo[a]));
    }
};

// distance from a point to a segment in 2D
double SegmentDistance2D(double px, double py, double ax, double ay, double bx, double by)
{
    const double dx = bx - ax, dy = by - ay;
    const double wx = px - ax, wy = py - ay;
    const double dd = dx * dx + dy * dy;
    const double t = dd > 0 ? std::clamp((wx * dx + wy * dy) / dd, 0.0, 1.0) : 0;
    const double ex = wx - t * dx, ey = wy - t * dy;
    return std::sqrt(ex * ex + ey * ey);
}

// distance between two segments in 3D
double SegmentDistance(const double a0[3], const double a1[3], const double b0[3], const double b1[3])
{
    double u[3], v[3], w[3];
    for (int k = 0; k < 3; k++) {
        u[k] = a1[k] - a0[k];
        v[k] = b1[k] - b0[k];
        w[k] = a0[k] - b0[k];
    }
    const double a = u[0] * u[0] + u[1] * u[1] + u[2] * u[2];
    const double b = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    const double c = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    const double d = u[0] * w[0] + u[1] * w[1] + u[2] * w[2];
    const double e = v[0] * w[0] + v[1] * w[1] + v[2] * w[2];
    const double det = a * c - b * b;
    double s = 0, t = 0;
    if (a <= 1e-24 && c <= 1e-24) {
        s = t = 0;
    }
    else if (a <= 1e-24) {
        t = std::clamp(e / c, 0.0, 1.0);
    }
    else if (c <= 1e-24) {
        s = std::clamp(-d / a, 0.0, 1.0);
    }
    else {
        s = det > 1e-24 * a * c ? std::clamp((b * e - c * d) / det, 0.0, 1.0) : 0.0;
        t = (b * s + e) / c;
        if (t < 0) {
            t = 0;
            s = std::clamp(-d / a, 0.0, 1.0);
        }
        else if (t > 1) {
            t = 1;
            s = std::clamp((b - d) / a, 0.0, 1.0);
        }
    }
    double dist = 0;
    for (int k = 0; k < 3; k++) {
        const double x = w[k] + s * u[k] - t * v[k];
        dist += x * x;
    }
    return std::sqrt(dist);
}

// Tables of a tool for the planner, by distance from the axis (rounded up, conservative)
struct ToolTable
{
    const Tool* tool = nullptr;
    double step = 1;
    std::vector<double> maxBottom;  // the largest bottom height up to the distance
    std::vector<double> covered;    // height above the tip up to which the tool fills the distance
    double MaxBottom(double rho) const
    {
        const size_t i = std::min(maxBottom.size() - 1, (size_t)std::ceil(rho / step));
        return maxBottom[i];
    }
    double Covered(double rho) const
    {
        const size_t i = std::min(covered.size() - 1, (size_t)std::ceil(rho / step));
        return covered[i];
    }
    void Init(const Tool& t)
    {
        tool = &t;
        const int n = 64;
        step = t.Radius() / n;
        maxBottom.resize(n + 1);
        covered.resize(n + 1);
        // heights to check the radius at
        std::vector<double> zs;
        for (const auto& f : t.Frustums()) {
            zs.push_back(f.z0);
            zs.push_back(f.z1);
        }
        for (const auto& s : t.Spheres()) {
            zs.push_back(s.zc);
        }
        for (int i = 0; i <= 256; i++) {
            zs.push_back(t.MinZ() + (t.MaxZ() - t.MinZ()) * i / 256);
        }
        std::sort(zs.begin(), zs.end());
        for (int i = 0; i <= n; i++) {
            const double rho = i * step;
            maxBottom[i] = t.MaxBottom(rho);
            // up from the bottom while the solid reaches the distance
            double h = maxBottom[i];
            for (double z : zs) {
                if (z <= maxBottom[i]) {
                    continue;
                }
                if (t.RadiusAt(z) < rho - 1e-9) {
                    break;
                }
                h = z;
            }
            covered[i] = h;
        }
    }
};

}  // namespace

void Segment::PositionAt(double t, double pos[3]) const
{
    if (arc) {
        const double angle = startAngle + t * sweep;
        pos[0] = center[0] - std::sin(angle) * radius;
        pos[1] = center[1] + std::cos(angle) * radius;
        pos[2] = start[2] + t * (end[2] - start[2]);
        return;
    }
    for (int k = 0; k < 3; k++) {
        pos[k] = start[k] + t * (end[k] - start[k]);
    }
}

void Segment::AxisAt(double t, double axis[3]) const
{
    double len = 0;
    for (int k = 0; k < 3; k++) {
        axis[k] = axis0[k] + t * (axis1[k] - axis0[k]);
        len += axis[k] * axis[k];
    }
    len = std::sqrt(len);
    for (int k = 0; k < 3; k++) {
        axis[k] = len > 0 ? axis[k] / len : (k == 2 ? 1.0 : 0.0);
    }
}

Simulation::Simulation() = default;

Simulation::~Simulation()
{
    Stop();
}

void Simulation::Stop()
{
    mStop = true;
    mWake.notify_all();
    if (mWorker.joinable()) {
        mWorker.join();
    }
    mStop = false;
}

void Simulation::SetNotify(std::function<void()> notify)
{
    std::lock_guard<std::mutex> lock(mMutex);
    mNotify = std::move(notify);
}

Simulation::Stats Simulation::GetStats() const
{
    std::lock_guard<std::mutex> lock(mMutex);
    return mStats;
}

void Simulation::Start(
    std::vector<std::shared_ptr<const Tool>> tools,
    std::vector<Segment> segments,
    const std::vector<float>& xyz,
    const std::vector<uint32_t>& triangles,
    float quality
)
{
    Stop();
    if (!mPool) {
        mPool = std::make_unique<ThreadPool>(0, true);
    }
    mTools = std::move(tools);
    mSegments = std::move(segments);
    mQuality = std::clamp(quality, 1.0f, 10.0f);
    mBase.assign(1, 0);
    for (const Segment& s : mSegments) {
        mBase.push_back(mBase.back() + std::max(1, s.steps));
    }
    mStats = Stats();
    mStockXyz = xyz;
    mStockTriangles = triangles;
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mPlanned = false;
        mTarget = 0;
        mFrontierStep = 0;
        mDisplayStep = -1;
    }
    mWorker = std::thread([this] { Worker(); });
}

void Simulation::Prepare()
{
    auto start = Clock::now();
    Plan(mStockXyz, mQuality);
    const double planMs = MsSince(start);
    start = Clock::now();
    mInitial.Init(mLayout);
    mInitial.SetMesh(mStockXyz, mStockTriangles, *mPool);
    mInitial.TakeDirty();
    const double stockMs = MsSince(start);

    const int leaves = mLayout->LeafCount();
    mFrontier.Init(mLayout);
    mFrontier.Restore(mInitial.Bricks());
    mFrontier.TakeDirty();
    mView.Init(mLayout);
    mView.Restore(mInitial.Bricks());
    mView.TakeDirty();
    mFrontierMotion = 0;
    mFrontierChanged.clear();
    mFrontierFlag.assign(leaves, 0);
    mViewMotion = 0;
    mViewCheckpoint = 0;
    mViewStep = -1;
    mViewChanged.clear();
    mViewFlag.assign(leaves, 0);
    mHistory.assign(leaves, {});
    mCheckpoints.assign(1, Checkpoint {0, {}});
    mCheckpointBytes = 0;
    std::function<void()> notify;
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mStats.planMs = planMs;
        mStats.stockMs = stockMs;
        mStats.motions = (long)mMotions.size();
        mStats.leaves = leaves;
        mFrontierStep = mMotions.empty() ? TotalSteps() : 0;
        mDisplay.Init(mLayout);
        mDisplay.Restore(mInitial.Bricks());
        mDisplayStep = 0;
        mPlanned = true;
        notify = mNotify;
    }
    if (notify) {
        notify();
    }
}

void Simulation::Plan(const std::vector<float>& xyz, float quality)
{
    for (int k = 0; k < 3; k++) {
        mStockLo[k] = 1e300;
        mStockHi[k] = -1e300;
    }
    for (size_t i = 0; i + 2 < xyz.size(); i += 3) {
        for (int k = 0; k < 3; k++) {
            mStockLo[k] = std::min(mStockLo[k], (double)xyz[i + k]);
            mStockHi[k] = std::max(mStockHi[k], (double)xyz[i + k]);
        }
    }
    if (xyz.empty()) {
        for (int k = 0; k < 3; k++) {
            mStockLo[k] = 0;
            mStockHi[k] = 1;
        }
    }
    const double q = (quality - 1) / 9;  // 0 to 1
    const double longest = std::max(
        {mStockHi[0] - mStockLo[0], mStockHi[1] - mStockLo[1], mStockHi[2] - mStockLo[2], 1e-3}
    );
    // cells of the stock away from the tools, and of the tools: a fraction of their features
    const double rootCell = longest / (32 * std::pow(8.0, q));
    const double fraction = 0.5 * std::pow(0.2, q);
    const double finest = std::max(1e-3, 0.002 * longest / 100);
    // Arcs and motions turning the axis are approximated within a fiftieth of the cells of the
    // features of their tool: far below what the cells show
    std::vector<double> tolerances(mTools.size(), 5e-4);
    for (size_t t = 0; t < mTools.size(); t++) {
        if (mTools[t] && mTools[t]->Cuts()) {
            const double cell = std::max(finest, fraction * mTools[t]->FeatureSize());
            tolerances[t] = std::clamp(cell / 50, 5e-4, 1e-2);
        }
    }

    // motions: arcs into chords
    mMotions.clear();
    mSegmentMotion.assign(1, 0);
    for (size_t si = 0; si < mSegments.size(); si++) {
        const Segment& seg = mSegments[si];
        const Tool* tool = seg.tool >= 0 && seg.tool < (int)mTools.size() ? mTools[seg.tool].get()
                                                                          : nullptr;
        if (tool && tool->Cuts()) {
            int pieces = 1;
            const double tolerance = tolerances[seg.tool];
            // arcs of tools along Z at a constant height are swept exactly, in pieces of up to
            // a quarter circle, other arcs in chords
            double axis0[3], axis1[3];
            seg.AxisAt(0, axis0);
            seg.AxisAt(1, axis1);
            const bool exactArc = seg.arc && seg.radius > 0 && seg.start[2] == seg.end[2]
                && std::abs(axis0[2]) > 1 - 1e-12 && axis0[2] * axis1[2] > 1 - 1e-12;
            if (exactArc) {
                pieces = std::max(1, (int)std::ceil(std::abs(seg.sweep) / (M_PI / 2) - 1e-9));
            }
            else if (seg.arc && seg.radius > 0) {
                const double maxAngle = tolerance < seg.radius
                    ? 2 * std::acos(1 - tolerance / seg.radius)
                    : 1.5;
                pieces = std::max(1, (int)std::ceil(std::abs(seg.sweep) / maxAngle));
            }
            for (int i = 0; i < pieces; i++) {
                Motion2 m;
                m.segment = (int)si;
                m.t0 = (double)i / pieces;
                m.t1 = (double)(i + 1) / pieces;
                m.motion.tool = tool;
                m.motion.tag = OperationTag(seg.operation);
                seg.PositionAt(m.t0, m.motion.p0);
                seg.PositionAt(m.t1, m.motion.p1);
                seg.AxisAt(m.t0, m.motion.a0);
                seg.AxisAt(m.t1, m.motion.a1);
                // axes of work planes along the axes of the stock, computed from angles, are
                // off by rounding: exactly along them, the cuts get the exact faces of the tool
                for (double* a : {m.motion.a0, m.motion.a1}) {
                    for (int k = 0; k < 3; k++) {
                        if (std::abs(a[k]) > 1 - 1e-12) {
                            const double sign = a[k] > 0 ? 1 : -1;
                            a[0] = a[1] = a[2] = 0;
                            a[k] = sign;
                        }
                    }
                }
                if (exactArc) {
                    m.motion.arc = true;
                    m.motion.center[0] = seg.center[0];
                    m.motion.center[1] = seg.center[1];
                    m.motion.radius = seg.radius;
                    m.motion.startAngle = seg.startAngle + m.t0 * seg.sweep;
                    m.motion.sweep = (m.t1 - m.t0) * seg.sweep;
                    m.motion.p1[2] = m.motion.p0[2];
                }
                m.motion.cutHeight = CuttingHeight(m.motion, mStockLo, mStockHi);
                m.motion.tolerance = tolerance;
                mMotions.push_back(m);
            }
        }
        mSegmentMotion.push_back((long)mMotions.size());
    }

    // levels of the tools
    std::vector<ToolTable> tables(mTools.size());
    int levels = 0;
    auto levelFor = [&](double feature) {
        const double cell = std::max(finest, fraction * feature);
        return std::clamp((int)std::ceil(std::log2(rootCell / cell) - 1e-9), 0, 10);
    };
    for (size_t t = 0; t < mTools.size(); t++) {
        if (mTools[t] && mTools[t]->Cuts()) {
            tables[t].Init(*mTools[t]);
            levels = std::max(levels, levelFor(0.02 * mTools[t]->Radius()));
        }
    }
    auto tableOf = [&](const Tool* tool) -> const ToolTable& {
        for (const ToolTable& t : tables) {
            if (t.tool == tool) {
                return t;
            }
        }
        return tables[0];
    };

    // Where every motion can reach material: the material top along aligned tool axes, followed
    // on height maps, lowered where tools without undercuts leave nothing above their bottom
    HeightMap maps[6];
    const double mapCell = rootCell;
    for (int i = 0; i < 6; i++) {
        maps[i].Init(i / 2, i % 2 ? -1 : 1, mStockLo, mStockHi, mapCell);
    }
    struct Request
    {
        double a[3], b[3], radius;
        int level;
    };
    std::vector<Request> requests;
    std::vector<uint8_t> reaches(mMotions.size(), 0);
    for (Motion2& m2 : mMotions) {
        Motion& m = m2.motion;
        const Tool& tool = *m.tool;
        double reach = std::min(m.cutHeight, tool.MaxZ());
        // an axis aligned with a height map, the same at both ends
        int map = -1;
        for (int k = 0; k < 3; k++) {
            if (std::abs(m.a0[k]) > 1 - 1e-9 && std::abs(m.a1[k] - m.a0[k]) < 1e-9) {
                map = 2 * k + (m.a0[k] > 0 ? 0 : 1);
            }
        }
        if (map >= 0) {
            HeightMap& hm = maps[map];
            const ArcGeometry arc = m.arc ? ArcGeometry(m) : ArcGeometry();
            const double tip0 = hm.sign * m.p0[hm.axis], tip1 = hm.sign * m.p1[hm.axis];
            const double tipLo = std::min(tip0, tip1), tipHi = std::max(tip0, tip1);
            const double R = tool.Radius();
            const double au = m.p0[hm.u], av = m.p0[hm.v], bu = m.p1[hm.u], bv = m.p1[hm.v];
            // the box of the path in the map
            double plo[2] = {std::min(au, bu), std::min(av, bv)},
                   phi[2] = {std::max(au, bu), std::max(av, bv)};
            if (m.arc) {
                // arcs are along Z: u is X, v is Y; the box of their circle, enough
                plo[0] = std::min(plo[0], m.center[0] - m.radius);
                phi[0] = std::max(phi[0], m.center[0] + m.radius);
                plo[1] = std::min(plo[1], m.center[1] - m.radius);
                phi[1] = std::max(phi[1], m.center[1] + m.radius);
            }
            const int i0 = std::max(0, (int)std::floor((plo[0] - R - hm.lo[0]) / hm.cell));
            const int i1 = std::min(hm.n[0] - 1, (int)std::floor((phi[0] + R - hm.lo[0]) / hm.cell));
            const int j0 = std::max(0, (int)std::floor((plo[1] - R - hm.lo[1]) / hm.cell));
            const int j1 = std::min(hm.n[1] - 1, (int)std::floor((phi[1] + R - hm.lo[1]) / hm.cell));
            const double half = 0.7071067811865476 * hm.cell;
            const ToolTable& table = tableOf(&tool);
            double top = -1e300;
            const double reachCell = R + half;
            for (int j = j0; j <= j1; j++) {
                // the cells of the row within reach of the path: the row through the capsule
                const double cv = hm.lo[1] + (j + 0.5) * hm.cell;
                double xlo = 1e300, xhi = -1e300;
                for (const double* e : {m.p0, m.p1}) {
                    const double dv = cv - e[hm.v];
                    if (std::abs(dv) <= reachCell) {
                        const double h = std::sqrt(reachCell * reachCell - dv * dv);
                        xlo = std::min(xlo, e[hm.u] - h);
                        xhi = std::max(xhi, e[hm.u] + h);
                    }
                }
                const double su = bu - au, sv = bv - av;
                const double len = std::sqrt(su * su + sv * sv);
                if (len > 1e-12 && std::abs(sv) > 1e-12) {
                    // the band of the segment: |cross| <= reach * len, between its ends
                    const double t0 = (cv - av) / sv;
                    const double width = std::abs(reachCell * len / sv);
                    const double x = au + t0 * su;
                    xlo = std::min(xlo, x - width);
                    xhi = std::max(xhi, x + width);
                }
                else if (len > 1e-12 && std::abs(cv - av) <= reachCell) {
                    xlo = std::min(xlo, std::min(au, bu) - reachCell);
                    xhi = std::max(xhi, std::max(au, bu) + reachCell);
                }
                const int ri0 = m.arc ? i0
                                      : std::max(i0, (int)std::floor((xlo - hm.lo[0]) / hm.cell));
                const int ri1 = m.arc ? i1
                                      : std::min(i1, (int)std::floor((xhi - hm.lo[0]) / hm.cell));
                for (int i = ri0; i <= ri1; i++) {
                    const double cu = hm.lo[0] + (i + 0.5) * hm.cell;
                    const double dist = m.arc ? arc.Distance(cu, cv)
                                              : SegmentDistance2D(cu, cv, au, av, bu, bv);
                    if (dist > reachCell) {
                        continue;
                    }
                    float& cellTop = hm.top[(size_t)j * hm.n[0] + i];
                    top = std::max(top, (double)cellTop);
                    // all material of the cell within the reach of the bottom of the tool, below
                    // the top of the tool there, is removed above that bottom
                    const double far = dist + half;
                    if (tool.SingleAxial() && far <= R) {
                        const double bottom = tipHi + table.MaxBottom(far);
                        if (cellTop <= tipLo + table.Covered(far) && bottom < cellTop) {
                            cellTop = (float)bottom;
                        }
                    }
                }
            }
            reach = std::min(reach, top - tipLo);
        }
        if (reach <= tool.MinZ()) {
            continue;  // above the material
        }
        reaches[&m2 - mMotions.data()] = 1;
        // the cells for the widest part of the tool in the material
        const double width = tool.RadiusBelow(reach - tool.MinZ());
        const int level = levelFor(std::max(width, 0.02 * tool.Radius()));
        if (level <= 0) {
            continue;
        }
        const double cell = rootCell / (double)(1 << level);
        const double chunk = std::max(width, cell * Cells);
        for (double z = tool.MinZ(); z < reach; z += chunk) {
            const double zTop = std::min(reach, z + chunk);
            const double zm = 0.5 * (z + zTop);
            Request r;
            for (int k = 0; k < 3; k++) {
                r.a[k] = m.p0[k] + zm * m.a0[k];
                r.b[k] = m.p1[k] + zm * m.a1[k];
            }
            // arcs: the capsule of the chord, wide enough for the arc
            const double sagitta = m.arc ? m.radius * (1 - std::cos(m.sweep / 2)) : 0;
            const double rr = tool.RadiusBelow(zTop - tool.MinZ()) + cell + sagitta;
            r.radius = std::sqrt(rr * rr + 0.25 * (zTop - z) * (zTop - z));
            r.level = level;
            requests.push_back(r);
        }
    }

    // Grooves: passes of a ball end beside each other leave ridges between them, whose sides
    // meet at an angle growing with their distance. Where it is more than 10 degrees, and the
    // ridges are higher than a tenth of the cells of the tool, the cells along the passes get an
    // eighth of their distance, so the ridges show: up to 2 levels finer than the tool at the
    // highest quality, none at the lowest. Passes are found by the paths of
    // the centers of the balls, in 3D, with any tool axis, from all passes of the tool.
    const int grooveLevels = (int)std::lround(2 * q);
    if (grooveLevels > 0) {
        for (size_t t = 0; t < mTools.size(); t++) {
            const Tool* tool = mTools[t].get();
            if (!tool || !tool->Cuts() || tool->Spheres().empty()) {
                continue;
            }
            const Tool::Sphere ball = tool->Spheres()[0];
            if (ball.zc - ball.radius > tool->MinZ() + 1e-3) {
                continue;  // not a ball end
            }
            const double R = ball.radius;
            // the center paths of its passes in a hash of cells of the size of the ball
            struct Path
            {
                double a[3], b[3];
                int motion;
            };
            std::vector<Path> paths;
            for (size_t i = 0; i < mMotions.size(); i++) {
                const Motion& m = mMotions[i].motion;
                if (m.tool != tool || !reaches[i]) {
                    continue;
                }
                Path path;
                for (int k = 0; k < 3; k++) {
                    path.a[k] = m.p0[k] + ball.zc * m.a0[k];
                    path.b[k] = m.p1[k] + ball.zc * m.a1[k];
                }
                path.motion = (int)i;
                paths.push_back(path);
            }
            if (paths.size() < 2) {
                continue;
            }
            const double hashCell = 2 * R;
            auto keyOf = [&](long x, long y, long z) {
                return ((uint64_t)(x & 0x1FFFFF)) | ((uint64_t)(y & 0x1FFFFF) << 21)
                    | ((uint64_t)(z & 0x1FFFFF) << 42);
            };
            std::unordered_map<uint64_t, std::vector<int>> hash;
            for (size_t i = 0; i < paths.size(); i++) {
                long lo[3], hi[3];
                for (int k = 0; k < 3; k++) {
                    lo[k] = (long)std::floor(std::min(paths[i].a[k], paths[i].b[k]) / hashCell);
                    hi[k] = (long)std::floor(std::max(paths[i].a[k], paths[i].b[k]) / hashCell);
                }
                for (long z = lo[2]; z <= hi[2]; z++) {
                    for (long y = lo[1]; y <= hi[1]; y++) {
                        for (long x = lo[0]; x <= hi[0]; x++) {
                            hash[keyOf(x, y, z)].push_back((int)i);
                        }
                    }
                }
            }
            const int toolLevel = levelFor(tool->FeatureSize());
            const double toolCell = rootCell / (double)(1 << toolLevel);
            std::vector<Request> found(paths.size());
            std::vector<uint8_t> has(paths.size(), 0);
            std::atomic<size_t> nextPath {0};
            mPool->Run([&](int) {
                std::vector<int> seen;
                for (size_t i; (i = nextPath.fetch_add(32)) < paths.size();) {
                    for (size_t pi = i; pi < std::min(paths.size(), i + 32); pi++) {
                        const Path& p = paths[pi];
                        long lo[3], hi[3];
                        for (int k = 0; k < 3; k++) {
                            lo[k] = (long)std::floor((std::min(p.a[k], p.b[k]) - 2 * R) / hashCell);
                            hi[k] = (long)std::floor((std::max(p.a[k], p.b[k]) + 2 * R) / hashCell);
                        }
                        // the nearest pass beside it, not touching it
                        double nearest = 2 * R;
                        seen.clear();
                        for (long z = lo[2]; z <= hi[2]; z++) {
                            for (long y = lo[1]; y <= hi[1]; y++) {
                                for (long x = lo[0]; x <= hi[0]; x++) {
                                    auto it = hash.find(keyOf(x, y, z));
                                    if (it == hash.end()) {
                                        continue;
                                    }
                                    for (int other : it->second) {
                                        if (other == (int)pi) {
                                            continue;
                                        }
                                        const double d
                                            = SegmentDistance(p.a, p.b, paths[other].a, paths[other].b);
                                        if (d > 0.1 * R && d < nearest) {
                                            nearest = d;
                                        }
                                    }
                                }
                            }
                        }
                        // ridges steeper than 10 degrees, higher than a tenth of the cells of
                        // the tool
                        const double ridge = R
                            - std::sqrt(std::max(0.0, R * R - nearest * nearest / 4));
                        if (nearest >= 2 * R || 2 * std::asin(nearest / (2 * R)) < 10 * M_PI / 180
                            || ridge < 0.1 * toolCell) {
                            continue;
                        }
                        const int level = std::min(
                            toolLevel + grooveLevels,
                            std::clamp((int)std::ceil(std::log2(rootCell / (nearest / 8)) - 1e-9), 0, 10)
                        );
                        if (level <= toolLevel) {
                            continue;
                        }
                        Request& r = found[pi];
                        std::copy(p.a, p.a + 3, r.a);
                        std::copy(p.b, p.b + 3, r.b);
                        r.radius = R + rootCell / (double)(1 << level);
                        r.level = level;
                        has[pi] = 1;
                    }
                }
            });
            for (size_t i = 0; i < paths.size(); i++) {
                if (has[i]) {
                    requests.push_back(found[i]);
                    levels = std::max(levels, found[i].level);
                }
            }
        }
    }

    // the layout
    // a cell of margin around the stock, so its faces are inside the layout
    double layoutLo[3], layoutHi[3];
    for (int k = 0; k < 3; k++) {
        layoutLo[k] = mStockLo[k] - rootCell;
        layoutHi[k] = mStockHi[k] + rootCell;
    }
    Layout::Builder builder(layoutLo, layoutHi, rootCell, levels, mPool->Size());
    std::atomic<size_t> next {0};
    mPool->Run([&](int thread) {
        for (size_t i; (i = next.fetch_add(64)) < requests.size();) {
            for (size_t k = i; k < std::min(requests.size(), i + 64); k++) {
                const Request& r = requests[k];
                builder.AddCapsule(thread, r.a, r.b, r.radius, r.level);
            }
        }
    });
    mLayout = builder.Build(*mPool);
}

void Simulation::Locate(long step, long& motions, long& partialMotion, double& partialT) const
{
    partialMotion = -1;
    partialT = 0;
    if (step >= TotalSteps()) {
        motions = (long)mMotions.size();
        return;
    }
    if (step <= 0) {
        motions = 0;
        return;
    }
    const size_t s = std::upper_bound(mBase.begin(), mBase.end(), step) - mBase.begin() - 1;
    const double t = mSegments[s].StepParam((int)(step - mBase[s]));
    long m = mSegmentMotion[s];
    while (m < mSegmentMotion[s + 1] && mMotions[m].t1 <= t) {
        m++;
    }
    motions = m;
    if (m < mSegmentMotion[s + 1] && t > mMotions[m].t0) {
        partialMotion = m;
        partialT = t;
    }
}

long Simulation::StepOfMotion(long motions) const
{
    if (motions >= (long)mMotions.size()) {
        return TotalSteps();
    }
    const Motion2& m = mMotions[motions];
    const Segment& seg = mSegments[m.segment];
    return mBase[m.segment] + (long)std::floor(m.t0 * std::max(1, seg.sweepSteps));
}

long Simulation::CutBatch(Grid& grid, long from, long to)
{
    const long end = std::min(to, from + std::max(1L, (long)mMotionsPerBatch));
    std::vector<Sweep> sweeps;
    sweeps.reserve(end - from);
    for (long i = from; i < end; i++) {
        sweeps.emplace_back(mMotions[i].motion);
    }
    std::vector<const Sweep*> ptrs;
    ptrs.reserve(sweeps.size());
    for (const Sweep& s : sweeps) {
        ptrs.push_back(&s);
    }
    const auto start = Clock::now();
    grid.Cut(ptrs, *mPool);
    const double ms = MsSince(start);
    // batches of about BatchMs
    if (ms < BatchMs / 2) {
        mMotionsPerBatch = std::min(65536.0, mMotionsPerBatch * 1.5);
    }
    else if (ms > BatchMs * 2) {
        mMotionsPerBatch = std::max(8.0, mMotionsPerBatch * 0.6);
    }
    return end;
}

void Simulation::TrackView()
{
    for (int leaf : mView.TakeDirty()) {
        if (!mViewFlag[leaf]) {
            mViewFlag[leaf] = 1;
            mViewChanged.push_back(leaf);
        }
    }
}

int Simulation::CheckpointAt(long motions) const
{
    int c = (int)(std::upper_bound(
                      mCheckpoints.begin(),
                      mCheckpoints.end(),
                      motions,
                      [](long m, const Checkpoint& ck) { return m < ck.motion; }
                  )
                  - mCheckpoints.begin())
        - 1;
    while (c > 0 && !mCheckpoints[c].live) {
        c--;
    }
    return std::max(c, 0);
}

long Simulation::RestoreView(long motions)
{
    const int c = CheckpointAt(motions);
    // the view can go on from where it is
    if (mViewMotion >= 0 && mViewMotion >= mCheckpoints[c].motion && mViewMotion <= motions) {
        return mViewMotion;
    }
    // the leaves that differ from the checkpoint: changed in the view since its checkpoint,
    // and between the checkpoints; all of them if the view lost its checkpoint
    TrackView();
    std::vector<int> leaves;
    leaves.swap(mViewChanged);
    for (int leaf : leaves) {
        mViewFlag[leaf] = 0;
    }
    if (mViewMotion < 0) {
        leaves.resize(mLayout->LeafCount());
        for (int l = 0; l < (int)leaves.size(); l++) {
            leaves[l] = l;
        }
    }
    else {
        for (int k = std::min(c, mViewCheckpoint) + 1; k <= std::max(c, mViewCheckpoint); k++) {
            leaves.insert(leaves.end(), mCheckpoints[k].changed.begin(), mCheckpoints[k].changed.end());
        }
    }
    for (int leaf : leaves) {
        const auto& h = mHistory[leaf];
        auto it = std::upper_bound(h.begin(), h.end(), c, [](int ck, const auto& e) {
            return ck < e.first;
        });
        mView.SetBrick(leaf, it == h.begin() ? mInitial.GetBrick(leaf) : std::prev(it)->second);
    }
    mView.TakeDirty();
    mViewCheckpoint = c;
    mViewMotion = mCheckpoints[c].motion;
    return mViewMotion;
}

void Simulation::ThinCheckpoints()
{
    auto bytes = [](const Grid::BrickRef& b) {
        return b && b != Grid::Solid() ? (long)(sizeof(Brick) + b->spans.size() * sizeof(Span)) : 0L;
    };
    std::vector<int> live;
    for (int k = 0; k < (int)mCheckpoints.size(); k++) {
        if (mCheckpoints[k].live) {
            live.push_back(k);
        }
    }
    // keep the first and the last
    for (size_t i = 1; i + 1 < live.size(); i += 2) {
        const int k = live[i], next = live[i + 1];
        Checkpoint& dead = mCheckpoints[k];
        Checkpoint& keep = mCheckpoints[next];
        for (int leaf : dead.changed) {
            auto& h = mHistory[leaf];
            auto it = std::find_if(h.begin(), h.end(), [&](const auto& e) { return e.first == k; });
            if (it == h.end()) {
                continue;
            }
            auto after = std::next(it);
            if (after != h.end() && after->first <= next) {
                // changed again before the next checkpoint: never restored
                mCheckpointBytes -= bytes(it->second);
                h.erase(it);
            }
            else {
                it->first = next;
                keep.changed.push_back(leaf);
            }
        }
        std::sort(keep.changed.begin(), keep.changed.end());
        keep.changed.erase(std::unique(keep.changed.begin(), keep.changed.end()), keep.changed.end());
        dead.changed.clear();
        dead.changed.shrink_to_fit();
        dead.live = false;
        if (mViewCheckpoint == k) {
            mViewMotion = -1;  // restored completely next time
        }
    }
}

void Simulation::SeekView(long target, long motions, long partialMotion, double partialT)
{
    long at = RestoreView(motions);
    while (at < motions && !mStop) {
        at = CutBatch(mView, at, motions);
    }
    if (mStop) {
        return;
    }
    mViewMotion = motions;
    TrackView();
    if (partialMotion < 0) {
        Publish(target, mView);
    }
    else {
        // the part of the motion up to the target, in the published stock only
        const Motion2& m2 = mMotions[partialMotion];
        const Segment& seg = mSegments[m2.segment];
        Motion m = m2.motion;
        seg.PositionAt(partialT, m.p1);
        seg.AxisAt(partialT, m.a1);
        if (m.arc) {
            m.sweep = (partialT - m2.t0) * seg.sweep;
            m.p1[2] = m.p0[2];
            m.a1[0] = m.a0[0];
            m.a1[1] = m.a0[1];
            m.a1[2] = m.a0[2];
        }
        const Sweep sweep(m);
        std::vector<std::pair<int, Grid::BrickRef>> undo;
        mView.Cut({&sweep}, *mPool, &undo);
        Publish(target, mView);
        for (auto& u : undo) {
            mView.SetBrick(u.first, std::move(u.second));
        }
        mView.TakeDirty();
    }
    mViewStep = target;
}

void Simulation::AddCheckpoint()
{
    const int index = (int)mCheckpoints.size();
    Checkpoint ck;
    ck.motion = mFrontierMotion;
    ck.changed.swap(mFrontierChanged);
    for (int leaf : ck.changed) {
        mFrontierFlag[leaf] = 0;
        const Grid::BrickRef& b = mFrontier.GetBrick(leaf);
        mHistory[leaf].emplace_back(index, b);
        if (b && b != Grid::Solid()) {
            mCheckpointBytes += (long)(sizeof(Brick) + b->spans.size() * sizeof(Span));
        }
    }
    mCheckpoints.push_back(std::move(ck));
    if (mCheckpointBytes > CheckpointBudget) {
        ThinCheckpoints();
    }
}

void Simulation::Publish(long step, const Grid& grid)
{
    std::function<void()> notify;
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mDisplay.Restore(grid.Bricks());
        mDisplayStep = step;
        notify = mNotify;
    }
    if (notify) {
        notify();
    }
}

long Simulation::TakeDisplay(Grid& grid, std::vector<int>& changed)
{
    std::lock_guard<std::mutex> lock(mMutex);
    if (!mPlanned) {
        return -1;
    }
    for (int leaf : mDisplay.TakeDirty()) {
        grid.SetBrick(leaf, mDisplay.GetBrick(leaf));
        changed.push_back(leaf);
    }
    return mDisplayStep;
}

void Simulation::SetTarget(long step)
{
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mTarget == step) {
            return;
        }
        mTarget = step;
    }
    mWake.notify_all();
}

bool Simulation::Pending() const
{
    std::lock_guard<std::mutex> lock(mMutex);
    return mDisplayStep != mTarget;
}

long Simulation::FrontierStep() const
{
    std::lock_guard<std::mutex> lock(mMutex);
    return mFrontierStep;
}

void Simulation::WaitFrontier() const
{
    std::unique_lock<std::mutex> lock(mMutex);
    mFrontierDone.wait(lock, [&] { return mFrontierStep >= TotalSteps(); });
}

void Simulation::Worker()
{
    ThreadPool::LowerCurrentThreadPriority();
    const auto start = Clock::now();
    Prepare();
    double sinceCheckpoint = 0;
    auto lastPublish = Clock::now();
    bool frontierDone = false;
    while (!mStop) {
        long target;
        {
            std::lock_guard<std::mutex> lock(mMutex);
            target = mTarget;
        }
        long motions, partialMotion;
        double partialT;
        Locate(target, motions, partialMotion, partialT);
        if (mViewStep != target && motions <= mFrontierMotion) {
            SeekView(target, motions, partialMotion, partialT);
            continue;
        }
        if (!frontierDone) {
            const auto t0 = Clock::now();
            mFrontierMotion = CutBatch(mFrontier, mFrontierMotion, (long)mMotions.size());
            sinceCheckpoint += MsSince(t0);
            for (int leaf : mFrontier.TakeDirty()) {
                if (!mFrontierFlag[leaf]) {
                    mFrontierFlag[leaf] = 1;
                    mFrontierChanged.push_back(leaf);
                }
            }
            frontierDone = mFrontierMotion >= (long)mMotions.size();
            if (sinceCheckpoint >= CheckpointMs || frontierDone) {
                AddCheckpoint();
                sinceCheckpoint = 0;
            }
            const long frontierStep = frontierDone ? TotalSteps() : StepOfMotion(mFrontierMotion);
            {
                std::lock_guard<std::mutex> lock(mMutex);
                mFrontierStep = frontierStep;
                if (frontierDone) {
                    mStats.simulateMs = MsSince(start);
                    mStats.checkpoints = (long)mCheckpoints.size();
                    mStats.checkpointBytes = mCheckpointBytes;
                }
            }
            if (frontierDone) {
                mFrontierDone.notify_all();
                if (const char* log = std::getenv("CAM_SIM_LOG")) {
                    FILE* f = std::fopen(log, "a");
                    std::fprintf(
                        f ? f : stderr,
                        "CAM simulation: plan %.0f ms, stock %.0f ms, done after %.0f ms, %ld "
                        "motions, "
                        "%d leaves, %zu checkpoints, %.0f MB\n",
                        mStats.planMs,
                        mStats.stockMs,
                        mStats.simulateMs,
                        mStats.motions,
                        mLayout->LeafCount(),
                        mCheckpoints.size(),
                        mCheckpointBytes / 1e6
                    );
                    if (f) {
                        std::fclose(f);
                    }
                }
            }
            // while the target is ahead, show how far the frontier got
            if (motions > mFrontierMotion && MsSince(lastPublish) > ProgressMs) {
                Publish(frontierStep, mFrontier);
                lastPublish = Clock::now();
                mViewStep = -1;
            }
            continue;
        }
        std::unique_lock<std::mutex> lock(mMutex);
        mWake.wait(lock, [&] { return mStop || mTarget != target; });
    }
}

}  // namespace CAMSimulator::Material
