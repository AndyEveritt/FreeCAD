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

// Work (cells) between two snapshots of the background simulation, and between checks of the
// time budget in Update
constexpr double CheckpointWork = 2e6;
constexpr double UpdateChunkWork = 1e5;
// Memory of snapshot tiles after which every other snapshot is dropped
constexpr long CheckpointMemory = 512L * 1024 * 1024;

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
    float cellSize
)
{
    Stop();
    mTools = std::move(tools);
    mSegments = std::move(segments);
    mCheckpoints.clear();

    // grid covering the stock
    float minx = 1e30f, maxx = -1e30f, miny = 1e30f, maxy = -1e30f;
    for (size_t i = 0; i + 2 < xyz.size(); i += 3) {
        minx = std::min(minx, xyz[i]);
        maxx = std::max(maxx, xyz[i]);
        miny = std::min(miny, xyz[i + 1]);
        maxy = std::max(maxy, xyz[i + 1]);
    }
    if (minx > maxx) {
        minx = maxx = miny = maxy = 0;
    }
    const int nx = std::max(1, (int)std::ceil((maxx - minx) / cellSize));
    const int ny = std::max(1, (int)std::ceil((maxy - miny) / cellSize));
    const float x0 = (minx + maxx) / 2 - nx * cellSize / 2;
    const float y0 = (miny + maxy) / 2 - ny * cellSize / 2;
    mInitial.Init(x0, y0, cellSize, nx, ny);
    mInitial.SetMesh(xyz, triangles, mPool);
    mInitial.Freeze();
    mInitial.TakeDirtyTiles();
    mStock.Init(x0, y0, cellSize, nx, ny);
    mStock.Restore(mInitial.Tiles());
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

void DexelSimulation::SetTarget(long step)
{
    mTarget = std::clamp(step, 0L, TotalSteps());
}

void DexelSimulation::AppendMoves(long from, long to, std::vector<CutMove>& moves) const
{
    if (to <= from || mSegments.empty()) {
        return;
    }
    const float tolerance = mInitial.CellSize() * 0.25f;
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
    stock.Init(mInitial.X0(), mInitial.Y0(), mInitial.CellSize(), mInitial.NX(), mInitial.NY());
    stock.Restore(mInitial.Tiles());
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
        memory += stock.TakeNewTileBytes();

        std::lock_guard<std::mutex> lock(mMutex);
        mCheckpoints.push_back({step, stock.Tiles()});
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
            best = it->tiles;
        }
    }
    if (mTarget < mCurrent || bestStep > mCurrent) {
        if (bestStep >= 0 && bestStep <= mTarget) {
            mStock.Restore(best);
            mCurrent = bestStep;
        }
        else {
            mStock.Restore(mInitial.Tiles());
            mCurrent = 0;
        }
        changed = true;
    }

    std::vector<CutMove> moves;
    while (mCurrent < mTarget) {
        const long next = std::min(mTarget, AdvanceByWork(mCurrent, (long)UpdateChunkWork));
        moves.clear();
        AppendMoves(mCurrent, next, moves);
        mStock.Cut(moves, mPool);
        mCurrent = next;
        changed = true;
        if (Clock::now() > deadline) {
            break;
        }
    }
    return changed;
}

}  // namespace CAMSimulator
