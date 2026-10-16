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

#include "MaterialRenderer.h"

#include "GlUtils.h"
#include "Shader.h"
#include "SimDisplay.h"
#include "ThreadPool.h"

#include <algorithm>
#include <cstddef>

// include this last as the defines can mess up other includes
#include "OpenGlWrapper.h"

namespace CAMSimulator::Material
{

namespace
{
// mesh units per GPU buffer
constexpr int UnitsPerChunk = 256;
}  // namespace

View::View() = default;

View::~View()
{
    Stop();
    FreeResources();
}

void View::Start(
    std::vector<std::shared_ptr<const Tool>> tools,
    std::vector<Segment> segments,
    const std::vector<float>& xyz,
    const std::vector<uint32_t>& triangles,
    float quality
)
{
    Stop();
    FreeResources();
    if (!mPool) {
        mPool = std::make_unique<ThreadPool>(0, true);
    }
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mReady.clear();
        mReadyStep = -1;
        mWork = false;
        mBusy = false;
        mTarget = 0;
    }
    mUnits.clear();
    mChunks.clear();
    mShownStep = -1;
    mSimulation.SetNotify([this] {
        {
            std::lock_guard<std::mutex> lock(mMutex);
            mWork = true;
        }
        mWake.notify_all();
    });
    mSimulation.Start(std::move(tools), std::move(segments), xyz, triangles, quality);
    mStop = false;
    mThread = std::thread([this] { MesherLoop(); });
    mStarted = true;
}

void View::Stop()
{
    mSimulation.Stop();
    mStop = true;
    mWake.notify_all();
    if (mThread.joinable()) {
        mThread.join();
    }
    mStop = false;
    mStarted = false;
}

void View::SetTarget(long step)
{
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mTarget = step;
    }
    mSimulation.SetTarget(step);
}

void View::SetMeasureThickness(bool b)
{
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mThickness == b) {
            return;
        }
        mThickness = b;
        mWork = true;
    }
    mWake.notify_all();
}

void View::MesherLoop()
{
    ThreadPool::LowerCurrentThreadPriority();
    std::shared_ptr<const Layout> layout;
    bool thickness = false;
    std::vector<int> changed;
    while (!mStop) {
        {
            std::unique_lock<std::mutex> lock(mMutex);
            mWake.wait(lock, [&] { return mStop || mWork; });
            if (mStop) {
                return;
            }
            mWork = false;
            mBusy = true;
            thickness = mThickness;
        }
        if (!layout) {
            layout = mSimulation.GetLayout();
            if (layout) {
                mMesher.Reset(layout, *mPool);
                mGrid.Init(layout);
            }
        }
        if (layout) {
            changed.clear();
            const long step = mSimulation.TakeDisplay(mGrid, changed);
            if (thickness != mMesher.MeasuresThickness()) {
                mMesher.SetMeasureThickness(thickness);
            }
            const std::vector<int> units = mMesher.Update(mGrid, changed, *mPool);
            std::map<int, UnitMesh> meshes;
            for (int u : units) {
                meshes.emplace(u, mMesher.Mesh(u));
            }
            std::lock_guard<std::mutex> lock(mMutex);
            for (auto& [u, mesh] : meshes) {
                mReady[u] = std::move(mesh);
            }
            mReadyStep = step;
        }
        std::lock_guard<std::mutex> lock(mMutex);
        mBusy = false;
    }
}

bool View::HasNewMesh() const
{
    std::lock_guard<std::mutex> lock(mMutex);
    return mReadyStep >= 0 && (!mReady.empty() || mReadyStep != mShownStep);
}

bool View::Pending() const
{
    if (!mStarted) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mMutex);
    return mShownStep != mTarget || mBusy || mWork || !mReady.empty() || mReadyStep != mShownStep;
}

bool View::Update()
{
    std::map<int, UnitMesh> ready;
    long step;
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mReadyStep < 0 || (mReady.empty() && mReadyStep == mShownStep)) {
            return false;
        }
        ready.swap(mReady);
        step = mReadyStep;
    }
    if (mUnits.empty()) {
        const std::shared_ptr<const Layout> layout = mSimulation.GetLayout();
        if (!layout) {
            return false;
        }
        mUnits.resize(layout->UnitCount());
        for (int first = 0; first < layout->UnitCount(); first += UnitsPerChunk) {
            Chunk c;
            c.firstUnit = first;
            c.endUnit = std::min(layout->UnitCount(), first + UnitsPerChunk);
            mChunks.push_back(c);
        }
    }
    for (auto& [unit, mesh] : ready) {
        mUnits[unit] = std::move(mesh);
        mChunks[unit / UnitsPerChunk].changed = true;
    }
    for (Chunk& chunk : mChunks) {
        if (chunk.changed) {
            Upload(chunk);
        }
    }
    mShownStep = step;
    return true;
}

void View::Upload(Chunk& chunk)
{
    chunk.changed = false;
    std::vector<MeshVertex> vertices;
    std::vector<uint32_t> indices;
    size_t nv = 0, ni = 0;
    for (int u = chunk.firstUnit; u < chunk.endUnit; u++) {
        nv += mUnits[u].vertices.size();
        ni += mUnits[u].stock.size() + mUnits[u].cut.size();
    }
    vertices.reserve(nv);
    indices.reserve(ni);
    // stock faces first, then cut faces
    for (int pass = 0; pass < 2; pass++) {
        size_t base = 0;
        for (int u = chunk.firstUnit; u < chunk.endUnit; u++) {
            const UnitMesh& mesh = mUnits[u];
            for (uint32_t index : pass == 0 ? mesh.stock : mesh.cut) {
                indices.push_back((uint32_t)(base + index));
            }
            base += mesh.vertices.size();
        }
        if (pass == 0) {
            chunk.stockCount = (int)indices.size();
        }
    }
    chunk.cutCount = (int)indices.size() - chunk.stockCount;
    for (int u = chunk.firstUnit; u < chunk.endUnit; u++) {
        vertices.insert(vertices.end(), mUnits[u].vertices.begin(), mUnits[u].vertices.end());
    }
    if (vertices.empty()) {
        chunk.stockCount = chunk.cutCount = 0;
        return;
    }
    if (chunk.vbo == 0) {
        glGenBuffers(1, &chunk.vbo);
        glGenBuffers(1, &chunk.ibo);
    }
    glBindBuffer(GL_ARRAY_BUFFER, chunk.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(MeshVertex), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, chunk.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32_t), indices.data(), GL_STATIC_DRAW);
}

void View::Draw(const Chunk& chunk, bool cut) const
{
    const int count = cut ? chunk.cutCount : chunk.stockCount;
    if (count == 0 || chunk.vbo == 0) {
        return;
    }
    glBindBuffer(GL_ARRAY_BUFFER, chunk.vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_BYTE, GL_TRUE, sizeof(MeshVertex), (void*)offsetof(MeshVertex, nx));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_UNSIGNED_BYTE, GL_FALSE, sizeof(MeshVertex), (void*)offsetof(MeshVertex, tag));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), (void*)offsetof(MeshVertex, thickness));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, chunk.ibo);
    const size_t offset = cut ? chunk.stockCount * sizeof(uint32_t) : 0;
    glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, (void*)offset);
    // other meshes only have positions and normals
    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(3);
}

void View::Render(
    SimDisplay& display,
    const float stockColor[3],
    const float cutColor[3],
    const DexelColoring& coloring
)
{
    const vec3 colors[2] = {
        {stockColor[0], stockColor[1], stockColor[2]},
        {cutColor[0], cutColor[1], cutColor[2]},
    };
    for (int pass = 0; pass < 2; pass++) {
        display.StartDexelGeometryPass(colors[pass], coloring);
        CurrentShader->UpdateModelMat(identityMat, identityMat);
        for (const Chunk& chunk : mChunks) {
            Draw(chunk, pass == 1);
        }
    }
}

void View::FreeResources()
{
    for (Chunk& chunk : mChunks) {
        GLDELETE_BUFFER(chunk.vbo);
        GLDELETE_BUFFER(chunk.ibo);
    }
    mChunks.clear();
    mUnits.clear();
}

long View::TriangleCount() const
{
    long count = 0;
    for (const Chunk& chunk : mChunks) {
        count += (chunk.stockCount + chunk.cutCount) / 3;
    }
    return count;
}

}  // namespace CAMSimulator::Material
