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
#include <cmath>
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

void View::SetDeviation(std::shared_ptr<const ModelDistance> model)
{
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mModel == model) {
            return;
        }
        mModel = std::move(model);
        mWork = true;
    }
    mWake.notify_all();
}

void View::MesherLoop()
{
    ThreadPool::LowerCurrentThreadPriority();
    std::shared_ptr<const Layout> layout;
    bool thickness = false;
    std::shared_ptr<const ModelDistance> model;
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
            model = mModel;
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
            mMesher.SetDeviation(model);
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
            if (mKeepBricks) {
                mReadyBricks = std::make_shared<const Grid::Snapshot>(mGrid.Bricks());
            }
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
    std::shared_ptr<const Grid::Snapshot> bricks;
    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mReadyStep < 0 || (mReady.empty() && mReadyStep == mShownStep)) {
            return false;
        }
        ready.swap(mReady);
        step = mReadyStep;
        bricks = mReadyBricks;
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
    if (bricks && bricks != mShownBricks) {
        mShownBricks = bricks;
        BuildDexels();
    }
    return true;
}

void View::SetInspect(Inspect inspect, float normalLength)
{
    if (inspect == mInspect && normalLength == mNormalLength) {
        return;
    }
    mInspect = inspect;
    mNormalLength = normalLength;
    for (Chunk& chunk : mChunks) {
        BuildInspection(chunk);
    }
    {
        // the dexels from the bricks of the meshes, all of them meshed again for those
        std::lock_guard<std::mutex> lock(mMutex);
        mKeepBricks = inspect == Inspect::Dexels;
        if (mKeepBricks && !mReadyBricks) {
            mWork = true;
        }
        if (!mKeepBricks) {
            mReadyBricks.reset();
        }
    }
    if (inspect != Inspect::Dexels) {
        mShownBricks.reset();
    }
    mWake.notify_all();
    BuildDexels();
}

void View::InspectAt(const float point[3])
{
    std::copy(point, point + 3, mInspectPoint);
    mHasInspectPoint = true;
    BuildDexels();
}

// The edges of the triangles and the normals of the vertices of a chunk
void View::BuildInspection(Chunk& chunk)
{
    if (mInspect != Inspect::Mesh) {
        GLDELETE_BUFFER(chunk.edgeIbo);
        chunk.edgeCount = 0;
    }
    if (mInspect != Inspect::Normals) {
        GLDELETE_BUFFER(chunk.normalVbo);
        chunk.normalCount = 0;
    }
    if (mInspect == Inspect::Mesh) {
        std::vector<uint32_t> edges;
        size_t base = 0;
        for (int u = chunk.firstUnit; u < chunk.endUnit; u++) {
            const UnitMesh& mesh = mUnits[u];
            for (const auto* list : {&mesh.stock, &mesh.cut}) {
                for (size_t t = 0; t + 2 < list->size(); t += 3) {
                    for (int k = 0; k < 3; k++) {
                        edges.push_back((uint32_t)(base + (*list)[t + k]));
                        edges.push_back((uint32_t)(base + (*list)[t + (k + 1) % 3]));
                    }
                }
            }
            base += mesh.vertices.size();
        }
        chunk.edgeCount = (int)edges.size();
        if (!edges.empty()) {
            if (chunk.edgeIbo == 0) {
                glGenBuffers(1, &chunk.edgeIbo);
            }
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, chunk.edgeIbo);
            glBufferData(
                GL_ELEMENT_ARRAY_BUFFER,
                edges.size() * sizeof(uint32_t),
                edges.data(),
                GL_STATIC_DRAW
            );
        }
    }
    if (mInspect == Inspect::Normals) {
        std::vector<float> lines;
        for (int u = chunk.firstUnit; u < chunk.endUnit; u++) {
            for (const MeshVertex& v : mUnits[u].vertices) {
                const float n[3] = {v.nx / 127.0f, v.ny / 127.0f, v.nz / 127.0f};
                lines.insert(
                    lines.end(),
                    {v.x,
                     v.y,
                     v.z,
                     v.x + mNormalLength * n[0],
                     v.y + mNormalLength * n[1],
                     v.z + mNormalLength * n[2]}
                );
            }
        }
        chunk.normalCount = (int)(lines.size() / 3);
        if (!lines.empty()) {
            if (chunk.normalVbo == 0) {
                glGenBuffers(1, &chunk.normalVbo);
            }
            glBindBuffer(GL_ARRAY_BUFFER, chunk.normalVbo);
            glBufferData(GL_ARRAY_BUFFER, lines.size() * sizeof(float), lines.data(), GL_STATIC_DRAW);
        }
    }
}

// The dexels of the bricks around the inspected point: the material along them, and their ends
// as small crosses, with a normal, without (made by a sharp edge of a tool), and not cut
void View::BuildDexels()
{
    std::fill(mDexelCounts, mDexelCounts + 4, 0);
    const std::shared_ptr<const Layout> layout = mSimulation.GetLayout();
    if (mInspect != Inspect::Dexels || !mHasInspectPoint || !mShownBricks || !layout) {
        GLDELETE_BUFFER(mDexelVbo);
        return;
    }
    // the brick at the point and the bricks around it, about 3 bricks of its size across
    std::vector<int> leaves;
    {
        int units[3];
        for (int k = 0; k < 3; k++) {
            units[k] = (int)std::floor((mInspectPoint[k] - layout->Origin()[k]) / layout->Unit());
        }
        const int leaf = layout->FindLeaf(units[0], units[1], units[2]);
        if (leaf < 0) {
            GLDELETE_BUFFER(mDexelVbo);
            return;
        }
        const double size = layout->GetLeaf(leaf).cell * Cells * layout->Unit();
        double lo[3], hi[3];
        for (int k = 0; k < 3; k++) {
            lo[k] = mInspectPoint[k] - 1.5 * size;
            hi[k] = mInspectPoint[k] + 1.5 * size;
        }
        layout->LeavesInBox(lo, hi, leaves);
    }
    Grid grid;
    grid.Init(layout);
    std::vector<float> groups[4];
    for (int leaf : leaves) {
        grid.SetBrick(leaf, (*mShownBricks)[leaf]);
        const double cell = layout->GetLeaf(leaf).cell * layout->Unit();
        const float tick = (float)(0.15 * cell);
        for (int line = 0; line < BrickLines; line++) {
            Span scratch;
            const Span* spans;
            const int n = grid.Line(leaf, line, spans, scratch);
            if (n == 0) {
                continue;
            }
            const int axis = line / AxisLines;
            double origin[3], rangeLo, rangeHi;
            grid.LineOrigin(leaf, line, origin);
            grid.LineRange(leaf, axis, rangeLo, rangeHi);
            auto point = [&](double s, float out[3]) {
                for (int k = 0; k < 3; k++) {
                    out[k] = (float)origin[k];
                }
                out[axis] = (float)s;
            };
            for (int i = 0; i < n; i++) {
                float a[3], b[3];
                point(spans[i].lo, a);
                point(spans[i].hi, b);
                groups[0].insert(groups[0].end(), {a[0], a[1], a[2], b[0], b[1], b[2]});
                for (int end = 0; end < 2; end++) {
                    const double s = end ? spans[i].hi : spans[i].lo;
                    // the ends of the lines are not surfaces
                    if (s <= rangeLo + 1e-6 || s >= rangeHi - 1e-6) {
                        continue;
                    }
                    const uint32_t packed = end ? spans[i].nhi : spans[i].nlo;
                    const int group = !IsCut(packed) ? 3 : HasNormal(packed) ? 1 : 2;
                    float c[3];
                    point(s, c);
                    for (int k = 1; k < 3; k++) {
                        const int across = (axis + k) % 3;
                        float p0[3] = {c[0], c[1], c[2]}, p1[3] = {c[0], c[1], c[2]};
                        p0[across] -= tick;
                        p1[across] += tick;
                        groups[group].insert(
                            groups[group].end(),
                            {p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]}
                        );
                    }
                }
            }
        }
    }
    std::vector<float> all;
    for (int g = 0; g < 4; g++) {
        mDexelCounts[g] = (int)(groups[g].size() / 3);
        all.insert(all.end(), groups[g].begin(), groups[g].end());
    }
    if (all.empty()) {
        GLDELETE_BUFFER(mDexelVbo);
        return;
    }
    if (mDexelVbo == 0) {
        glGenBuffers(1, &mDexelVbo);
    }
    glBindBuffer(GL_ARRAY_BUFFER, mDexelVbo);
    glBufferData(GL_ARRAY_BUFFER, all.size() * sizeof(float), all.data(), GL_STATIC_DRAW);
}

void View::RenderInspection(SimDisplay& display)
{
    if (mInspect == Inspect::None) {
        return;
    }
    glEnableVertexAttribArray(0);
    if (mInspect == Inspect::Mesh) {
        display.StartOverlayPass({0.1f, 0.1f, 0.12f});
        for (const Chunk& chunk : mChunks) {
            if (chunk.edgeCount == 0 || chunk.vbo == 0) {
                continue;
            }
            glBindBuffer(GL_ARRAY_BUFFER, chunk.vbo);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), (void*)0);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, chunk.edgeIbo);
            glDrawElements(GL_LINES, chunk.edgeCount, GL_UNSIGNED_INT, (void*)0);
        }
    }
    else if (mInspect == Inspect::Normals) {
        display.StartOverlayPass({0.95f, 0.8f, 0.1f});
        for (const Chunk& chunk : mChunks) {
            if (chunk.normalCount == 0) {
                continue;
            }
            glBindBuffer(GL_ARRAY_BUFFER, chunk.normalVbo);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
            glDrawArrays(GL_LINES, 0, chunk.normalCount);
        }
    }
    else if (mDexelVbo != 0) {
        // material blue, ends with a normal green, without red, not cut yellow
        const vec3 colors[4] = {
            {0.2f, 0.45f, 0.95f},
            {0.1f, 0.85f, 0.2f},
            {0.95f, 0.1f, 0.1f},
            {0.95f, 0.85f, 0.1f},
        };
        glBindBuffer(GL_ARRAY_BUFFER, mDexelVbo);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        int first = 0;
        for (int g = 0; g < 4; g++) {
            if (mDexelCounts[g] > 0) {
                display.StartOverlayPass(colors[g]);
                glDrawArrays(GL_LINES, first, mDexelCounts[g]);
            }
            first += mDexelCounts[g];
        }
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
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
    BuildInspection(chunk);
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
    glVertexAttribPointer(
        2,
        1,
        GL_UNSIGNED_SHORT,
        GL_FALSE,
        sizeof(MeshVertex),
        (void*)offsetof(MeshVertex, tag)
    );
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(
        3,
        1,
        GL_UNSIGNED_SHORT,
        GL_TRUE,
        sizeof(MeshVertex),
        (void*)offsetof(MeshVertex, thickness)
    );
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(
        4,
        1,
        GL_FLOAT,
        GL_FALSE,
        sizeof(MeshVertex),
        (void*)offsetof(MeshVertex, deviation)
    );
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, chunk.ibo);
    const size_t offset = cut ? chunk.stockCount * sizeof(uint32_t) : 0;
    glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, (void*)offset);
    // other meshes only have positions and normals
    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(3);
    glDisableVertexAttribArray(4);
}

void View::Render(
    SimDisplay& display,
    const float stockColor[3],
    const float cutColor[3],
    const StockColoring& coloring
)
{
    const vec3 colors[2] = {
        {stockColor[0], stockColor[1], stockColor[2]},
        {cutColor[0], cutColor[1], cutColor[2]},
    };
    for (int pass = 0; pass < 2; pass++) {
        display.StartStockGeometryPass(colors[pass], coloring);
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
        GLDELETE_BUFFER(chunk.edgeIbo);
        GLDELETE_BUFFER(chunk.normalVbo);
    }
    GLDELETE_BUFFER(mDexelVbo);
    mShownBricks.reset();
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
