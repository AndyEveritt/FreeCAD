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

#include "MaterialGrid.h"
#include "MaterialTool.h"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace CAMSimulator
{
class ThreadPool;
}

namespace CAMSimulator::Material
{

// One motion of the toolpath, divided into steps on the simulation timeline
struct Segment
{
    int tool = 0;  // index into the tools
    bool arc = false;
    bool rapid = false;    // a rapid move (G0)
    double start[3] = {};  // tool tip at the start and the end
    double end[3] = {};
    // arcs in the plane of the unit vectors arcU, arcV (XY by default): angle(t) = startAngle +
    // t * sweep, position = center - sin(angle) radius arcU + cos(angle) radius arcV, moving along
    // their normal arcU x arcV from start to end (helices)
    double center[3] = {};
    double radius = 0;
    double startAngle = 0;
    double sweep = 0;
    double arcU[3] = {1, 0, 0};
    double arcV[3] = {0, 1, 0};
    double axis0[3] = {0, 0, 1};  // tool axis at the start and the end
    double axis1[3] = {0, 0, 1};
    int steps = 1;       // steps on the timeline
    int sweepSteps = 1;  // steps of the motion, the parameter of step k is min(1, k / sweepSteps)
    int operation = -1;  // index of the operation, -1 for none

    double StepParam(int step) const
    {
        return std::min(1.0, (double)step / std::max(1, sweepSteps));
    }
    void PositionAt(double t, double pos[3]) const;
    void AxisAt(double t, double axis[3]) const;
};

// A problem the simulation finds on the toolpath
struct Event
{
    enum Kind
    {
        RapidIntoStock,  // a rapid move (G0) removes material
        ShankContact,    // the tool above its flutes meets material
        Gouge,           // a region of the final stock cut into the model
        MaterialLeft,    // a region of faces the toolpath cut, away from the model
        MachineLimit,    // the machine can't make the moves, see MachineEvents
    };
    Kind kind = RapidIntoStock;
    // the segments of the moves, one after the other, -1 for the regions of the final stock
    int firstSegment = -1;
    int lastSegment = -1;
    int operation = -1;              // regions: the operation that cut most of it
    double volume = 0;               // of the material met, mm3
    double position[3] = {0, 0, 0};  // the centroid of the material met or of the region
    // regions: the largest distance from the model, mm, the box of the regions, and their
    // number: the regions of one kind and operation make one event, at the deepest one
    double depth = 0;
    double lo[3] = {0, 0, 0};
    double hi[3] = {0, 0, 0};
    int count = 1;
    // machine limits: the linear axis whose travel is too short, what it needs and has (mm), or
    // none for a tool direction the rotary axes don't reach, then that direction
    std::string axis;
    double needed = 0;
    double available = 0;
    double direction[3] = {0, 0, 1};
};

class ModelDistance;

// Simulation of the stock material along the toolpath.
//
// Before simulating, the path is divided into motions (arcs into chords) and the layout of the
// stock is refined along the path of every tool to cells of a fraction of the size of the
// features it cuts, where it can reach the material.
//
// A worker thread simulates the whole toolpath once, from start to end, and keeps checkpoints of
// the stock at regular amounts of work, so the stock at any earlier position only needs the
// motions since the closest checkpoint before it. The worker serves the requested position first:
// the stock shown is never further than what is requested, and catches up with it while the
// simulation advances.
class Simulation
{
public:
    Simulation();
    ~Simulation();

    // The length of the flutes of the tools (by index, 0 or less for unknown) for the next start:
    // the tool above them meets the material, see Event::ShankContact
    void SetFluteHeights(std::vector<double> heights)
    {
        mFluteHeights = std::move(heights);
    }
    // How far the tool above its flutes is grown to find it rubbing walls cut by the flutes, mm
    static constexpr double ShankMargin = 0.01;

    // The model of the job, null for none, and the tolerance of the distance of the stock from it:
    // once the whole path is simulated, the faces the toolpath cut are measured against it, and
    // the regions cut into the model or left away from it are events (Gouge, MaterialLeft).
    // Can be set at any time, the regions are found again.
    void SetModel(std::shared_ptr<const ModelDistance> model, double tolerance);
    // Spacing of the points of the surface measured against the model, mm
    static constexpr double RegionSpacing = 0.25;

    // Start a simulation. xyz/triangles: closed stock mesh. quality: 1 (coarse) to 10 (fine).
    // Tools that don't cut (probes) are null.
    void Start(
        std::vector<std::shared_ptr<const Tool>> tools,
        std::vector<Segment> segments,
        const std::vector<float>& xyz,
        const std::vector<uint32_t>& triangles,
        float quality
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
    // The stock is wanted at this step
    void SetTarget(long step);
    // The latest stock ready for display, at the target or on the way to it: copies its bricks
    // that changed since the last call into 'grid', appends their leaves to 'changed', and
    // returns its step. 'grid' must have the layout of the simulation.
    long TakeDisplay(Grid& grid, std::vector<int>& changed);
    // True while the stock at the target isn't ready for display
    bool Pending() const;
    // Step up to which the whole path has been simulated once
    long FrontierStep() const;
    // The layout, once the worker planned it, null before
    std::shared_ptr<const Layout> GetLayout() const
    {
        std::lock_guard<std::mutex> lock(mMutex);
        return mPlanned ? mLayout : nullptr;
    }
    // Called from the worker when a new stock is ready
    void SetNotify(std::function<void()> notify);

    // Statistics of the last start
    struct Stats
    {
        double planMs = 0;
        double stockMs = 0;
        double simulateMs = 0;  // until the frontier reached the end
        long motions = 0;
        long leaves = 0;
        long checkpoints = 0;
        long checkpointBytes = 0;
    };
    Stats GetStats() const;

    // Blocks until the frontier reached the end (tests, benchmarks)
    void WaitFrontier() const;

    // Blocks until the frontier reached the end and the regions of the model are found
    void WaitEvents() const;
    // The problems found on the path up to the frontier, in the order of the path
    std::vector<Event> Events() const;
    // Count of the events, changes as they are found
    int EventCount() const;

private:
    struct Motion2
    {
        Motion motion;
        int segment;
        double t0, t1;       // range of the segment parameter
        bool shank = false;  // the tool above its flutes may meet the material
    };
    struct Checkpoint
    {
        long motion;               // motions done
        std::vector<int> changed;  // leaves changed since the previous live checkpoint
        bool live = true;          // dropped checkpoints are not restored to
    };

    void Plan(const std::vector<float>& xyz, float quality);
    // Plan, the initial stock, then simulate
    void Prepare();
    void Worker();
    // Cut a batch of the motions from 'from' on, up to 'to', returns where it got. With 'check',
    // the events the motions make are found.
    long CutBatch(Grid& grid, long from, long to, bool check = false);
    // A motion of a segment meets material in a way the kind of event reports
    void AddEvent(Event::Kind kind, int segment, const CutStats& stats);
    // The regions of the final stock (the frontier at the end) cut into the model or left
    void FindRegions();
    // Motions done and the partial segment parameter at a step
    void Locate(long step, long& motions, long& partialMotion, double& partialT) const;
    // The step at which a number of motions are done
    long StepOfMotion(long motions) const;
    // The view grid at the checkpoint at or before 'motions', or kept if it is between it and
    // 'motions'. Returns the motions done in the view.
    long RestoreView(long motions);
    void TrackView();
    void SeekView(long target, long motions, long partialMotion, double partialT);
    void AddCheckpoint();
    // Drop every other checkpoint, to stay within the memory budget
    void ThinCheckpoints();
    // The last live checkpoint at or before a number of motions
    int CheckpointAt(long motions) const;
    void Publish(long step, const Grid& grid);

    std::vector<std::shared_ptr<const Tool>> mTools;
    std::vector<double> mFluteHeights;
    // the tools above their flutes, grown by ShankMargin, null for none
    std::vector<std::shared_ptr<const Tool>> mShanks;
    std::vector<Segment> mSegments;
    std::vector<long> mBase;           // timeline step of the start of each segment, and the total
    std::vector<long> mSegmentMotion;  // first motion of each segment, and the total
    std::vector<Motion2> mMotions;
    std::shared_ptr<const Layout> mLayout;
    bool mPlanned = false;
    std::vector<float> mStockXyz;
    std::vector<uint32_t> mStockTriangles;
    double mStockLo[3] = {}, mStockHi[3] = {};
    float mQuality = 10;

    std::unique_ptr<ThreadPool> mPool;
    Grid mInitial;
    Grid mFrontier;
    Grid mView;
    long mFrontierMotion = 0;
    // leaves the frontier changed since the last checkpoint
    std::vector<int> mFrontierChanged;
    std::vector<uint8_t> mFrontierFlag;
    long mViewMotion = 0;     // motions done in the view grid
    int mViewCheckpoint = 0;  // checkpoint the view grid started from
    long mViewStep = -1;      // step published from the view grid
    // leaves the view changed since its checkpoint
    std::vector<int> mViewChanged;
    std::vector<uint8_t> mViewFlag;

    // per leaf: the bricks at checkpoints, (checkpoint index, brick)
    std::vector<std::vector<std::pair<int, Grid::BrickRef>>> mHistory;
    std::vector<Checkpoint> mCheckpoints;
    long mCheckpointBytes = 0;
    double mMotionsPerBatch = 256;

    // the stock for display, guarded by mMutex
    Grid mDisplay;
    long mDisplayStep = 0;

    std::thread mWorker;
    mutable std::mutex mMutex;
    mutable std::condition_variable mWake;
    mutable std::condition_variable mFrontierDone;
    std::atomic<bool> mStop {false};
    long mTarget = 0;
    long mFrontierStep = 0;
    std::function<void()> mNotify;
    Stats mStats;
    std::vector<Event> mEvents;  // guarded by mMutex
    // guarded by mMutex: the model, and the regions to find for it
    std::shared_ptr<const ModelDistance> mModel;
    double mTolerance = 0.05;
    bool mRegionsPending = false;
    bool mFindingRegions = false;
};

}  // namespace CAMSimulator::Material
