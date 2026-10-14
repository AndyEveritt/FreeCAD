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

#pragma once

#include "CutterProfile.h"
#include "DexelStock.h"
#include "ThreadPool.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace CAMSimulator
{

// One motion of the toolpath, divided into steps on the simulation timeline
struct CutSegment
{
    int tool = 0;  // index into the tools
    bool arc = false;
    float start[3] = {};  // tool tip at the start and the end
    float end[3] = {};
    // arcs: angle(t) = startAngle + t * sweep,
    // position = (center.x - sin(angle) * radius, center.y + cos(angle) * radius)
    float center[2] = {};
    float radius = 0;
    float startAngle = 0;
    float sweep = 0;
    float axis0[3] = {0, 0, 1};  // tool axis at the start and the end
    float axis1[3] = {0, 0, 1};
    int steps = 1;       // steps on the timeline
    int sweepSteps = 1;  // steps of the motion, the parameter of step k is min(1, k / sweepSteps)

    float StepParam(int step) const;
    void PositionAt(float t, float pos[3]) const;
    void AxisAt(float t, float axis[3]) const;
};

// Simulation of the stock material along the toolpath, with Z-dexels.
//
// The simulation moves to a target step on the timeline. A background thread simulates the whole
// toolpath once and keeps snapshots of the stock at regular intervals, so moving to any step only
// needs to simulate the steps since the closest snapshot before it.
class DexelSimulation
{
public:
    DexelSimulation();
    ~DexelSimulation();

    // Start a simulation. xyz/triangles: closed stock mesh. The grid covers the stock with cells
    // of the given size.
    void Start(
        std::vector<std::unique_ptr<CutterProfile>> tools,
        std::vector<CutSegment> segments,
        const std::vector<float>& xyz,
        const std::vector<uint32_t>& triangles,
        float cellSize
    );
    void Stop();

    long TotalSteps() const
    {
        return mBase.empty() ? 0 : mBase.back();
    }
    // Timeline step of a step of a segment
    long GlobalStep(int segment, int step) const
    {
        return mBase[segment] + step;
    }

    void SetTarget(long step);
    // Move towards the target for at most the given time. Returns true if the stock changed.
    bool Update(double budgetMs);
    bool Pending() const
    {
        return mCurrent != mTarget;
    }

    DexelStock& Stock()
    {
        return mStock;
    }
    const DexelStock& Initial() const
    {
        return mInitial;
    }
    ThreadPool& Pool()
    {
        return mPool;
    }

    // Moves for the timeline range (from, to]
    void AppendMoves(long from, long to, std::vector<CutMove>& moves) const;

private:
    struct Checkpoint
    {
        long step;
        DexelStock::Snapshot tiles;
    };

    void Worker();
    // Timeline step at which about 'work' more work than at 'from' is done
    long AdvanceByWork(long from, long work) const;
    double WorkAt(long step) const;

    std::vector<std::unique_ptr<CutterProfile>> mTools;
    std::vector<CutSegment> mSegments;
    std::vector<long> mBase;    // timeline step of the start of each segment, and the total
    std::vector<double> mWork;  // estimated work up to the start of each segment, and the total

    DexelStock mInitial;
    DexelStock mStock;
    long mCurrent = 0;
    long mTarget = 0;

    ThreadPool mPool;
    ThreadPool mWorkerPool;
    std::thread mWorker;
    std::atomic<bool> mStopWorker {false};
    mutable std::mutex mMutex;
    std::vector<Checkpoint> mCheckpoints;
};

}  // namespace CAMSimulator
