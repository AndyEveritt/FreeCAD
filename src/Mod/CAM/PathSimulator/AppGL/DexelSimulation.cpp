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
#include <chrono>
#include <cmath>

namespace CAMSimulator
{

namespace
{

using Clock = std::chrono::steady_clock;

// Work (lines) between two snapshots of the background simulation, and the least between checks
// of the time budget in Update
constexpr double CheckpointWork = 2e6;
constexpr double UpdateChunkWork = 1e5;
// Memory of snapshot bricks after which every other snapshot is dropped
constexpr long CheckpointMemory = 512L * 1024 * 1024;
// Leaves of the layout at most, the finest level is dropped until it fits
constexpr int MaxLeaves = 400000;

int WorkerThreads()
{
    return std::max(1, (int)std::thread::hardware_concurrency() / 2);
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
    std::vector<CutMove> moves;
    for (size_t s = 0; s < mSegments.size(); s++) {
        mBase.push_back(mBase.back() + mSegments[s].steps);
        moves.clear();
        AppendMoves(mBase[s], mBase[s + 1], moves);
        double work = 0;
        for (const CutMove& move : moves) {
            work += (double)mInitial.EstimateWork(move);
        }
        mWork.push_back(mWork.back() + work);
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
    // tool cuts: 0.5 to 0.1 times the size of the features of the tool.
    const float q = std::clamp(quality, 1.0f, 10.0f);
    const float baseCell = longest / (24 + 8 * q);
    const float relative = 0.5f * std::pow(0.2f, (q - 1) / 9);
    const float minCell = std::max(0.005f, longest / 4096);
    std::vector<float> toolCell;
    for (const auto& tool : mTools) {
        toolCell.push_back(std::clamp(relative * tool->FeatureSize(), minCell, baseCell));
    }
    // the finest level for the smallest tool, the others at the level closest to their cells
    float fineCell = baseCell;
    for (const CutSegment& seg : mSegments) {
        fineCell = std::min(fineCell, toolCell[seg.tool]);
    }
    int levels = std::clamp((int)std::floor(std::log2(baseCell / fineCell) + 1e-3), 0, 12);

    while (true) {
        auto layout = std::make_shared<DexelLayout>();
        layout->Init(lo, hi, fineCell, levels);
        for (const CutSegment& seg : mSegments) {
            const CutterProfile& tool = *mTools[seg.tool];
            const int level = std::clamp(
                levels - (int)std::lround(std::log2(toolCell[seg.tool] / fineCell)),
                0,
                levels
            );
            if (level == 0) {
                continue;
            }
            // pieces of arcs, the chords deviate at most a quarter of the tool radius
            const float tolerance = std::max(tool.Radius() / 4, fineCell);
            int pieces = 1;
            if (seg.arc && seg.radius > tolerance) {
                const float maxAngle = 2 * std::acos(1 - tolerance / seg.radius);
                pieces = std::max(1, (int)std::ceil(std::abs(seg.sweep) / maxAngle));
            }
            for (int i = 0; i < pieces; i++) {
                const float t0 = (float)i / pieces, t1 = (float)(i + 1) / pieces;
                float p0[3], p1[3], a0[3], a1[3];
                seg.PositionAt(t0, p0);
                seg.PositionAt(t1, p1);
                seg.AxisAt(t0, a0);
                seg.AxisAt(t1, a1);
                LayoutRegion region {};
                const float axisZ = std::min(a0[2], a1[2]);
                float length = tool.Height();
                if (axisZ >= 0.05f) {
                    const float side = tool.Radius() * std::sqrt(std::max(0.0f, 1 - axisZ * axisZ));
                    length = std::clamp((topZ - std::min(p0[2], p1[2]) + side) / axisZ, 0.0f, length);
                }
                if (a0[2] > 0.999999f && a1[2] > 0.999999f) {
                    std::copy(p0, p0 + 3, region.a);
                    std::copy(p1, p1 + 3, region.b);
                    region.radius = tool.Radius() + (seg.arc ? tolerance : 0);
                    region.vertical = true;
                    region.zlo = std::min(p0[2], p1[2]);
                    region.zhi = std::max(p0[2], p1[2]) + length;
                }
                else {
                    // spheres around the middle of the cutting part of the tool
                    float turn = 0;
                    for (int k = 0; k < 3; k++) {
                        region.a[k] = p0[k] + a0[k] * length / 2;
                        region.b[k] = p1[k] + a1[k] * length / 2;
                        turn = std::max(turn, std::abs(a1[k] - a0[k]));
                    }
                    region.radius = std::hypot(tool.Radius(), length / 2) + turn * turn * length / 4
                        + (seg.arc ? tolerance : 0);
                    region.vertical = false;
                }
                region.UpdateBounds();
                if (region.lo[2] > topZ) {
                    continue;  // above the stock
                }
                layout->Refine(region, level);
            }
        }
        layout->Finish();
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
        memory += stock.TakeNewBrickBytes();

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
        const double chunk = std::max(UpdateChunkWork, mWorkPerMs * remaining / 2);
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
