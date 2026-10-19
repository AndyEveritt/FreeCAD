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

#include "MaterialMesher.h"
#include "MaterialSimulation.h"

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace CAMSimulator
{
class SimDisplay;
struct StockColoring;
class ThreadPool;
}  // namespace CAMSimulator

namespace CAMSimulator::Material
{

// The simulation of the stock and its display: the simulation runs in its own thread, a mesher
// thread builds the mesh of every stock the simulation publishes, and the GUI thread only uploads
// the finished meshes and draws them. The meshes of a stock are shown all at once, so the shown
// surface never has cracks between old and new parts. Nothing of the simulation or the meshing
// runs in the GUI thread, so the view stays responsive while simulating.
class View
{
public:
    View();
    ~View();

    void Start(
        std::vector<std::shared_ptr<const Tool>> tools,
        std::vector<Segment> segments,
        const std::vector<float>& xyz,
        const std::vector<uint32_t>& triangles,
        float quality
    );
    void Stop();
    bool Started() const
    {
        return mStarted;
    }

    long TotalSteps() const
    {
        return mSimulation.TotalSteps();
    }
    long GlobalStep(int segment, int step) const
    {
        return mSimulation.GlobalStep(segment, step);
    }
    void SetTarget(long step);
    // Take finished meshes into the GPU buffers, in the GUI thread. True if the display changed.
    bool Update();
    // A finished mesh waits for Update
    bool HasNewMesh() const;
    // The stock shown is not the one at the target yet
    bool Pending() const;
    // Step of the stock shown
    long ShownStep() const
    {
        return mShownStep;
    }
    void Render(
        SimDisplay& display,
        const float stockColor[3],
        const float cutColor[3],
        const StockColoring& coloring
    );
    void SetMeasureThickness(bool b);
    void FreeResources();
    long TriangleCount() const;

private:
    struct Chunk
    {
        unsigned int vbo = 0;
        unsigned int ibo = 0;
        int stockCount = 0;
        int cutCount = 0;
        bool changed = false;
        int firstUnit = 0, endUnit = 0;
    };
    void MesherLoop();
    void Upload(Chunk& chunk);
    void Draw(const Chunk& chunk, bool cut) const;

    Simulation mSimulation;
    std::unique_ptr<ThreadPool> mPool;
    Mesher mMesher;
    Grid mGrid;  // the stock being meshed, a copy of the published one
    std::thread mThread;
    std::atomic<bool> mStop {false};
    bool mStarted = false;

    mutable std::mutex mMutex;
    std::condition_variable mWake;
    bool mWork = false;  // the simulation published, or the thickness option changed
    bool mBusy = false;  // meshing
    bool mThickness = false;
    std::map<int, UnitMesh> mReady;  // finished meshes by unit
    long mReadyStep = -1;
    long mTarget = 0;

    // GUI thread
    std::vector<UnitMesh> mUnits;
    std::vector<Chunk> mChunks;
    long mShownStep = -1;
};

}  // namespace CAMSimulator::Material
