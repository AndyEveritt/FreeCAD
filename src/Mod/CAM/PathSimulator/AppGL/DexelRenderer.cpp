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

#include "DexelRenderer.h"

#include "GlUtils.h"
#include "Shader.h"
#include "SimDisplay.h"
#include "ThreadPool.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>

// include this last as the defines can mess up other includes
#include "OpenGlWrapper.h"

namespace CAMSimulator
{

DexelRenderer::~DexelRenderer()
{
    FreeResources();
}

void DexelRenderer::FreeResources()
{
    for (auto& chunk : mChunks) {
        GLDELETE_BUFFER(chunk.vbo);
        GLDELETE_BUFFER(chunk.ibo);
    }
    mChunks.clear();
    mUnits.clear();
    mLayout.reset();
}

void DexelRenderer::Reset(const DexelStock& stock)
{
    FreeResources();
    mLayout = stock.LayoutRef();
    mUnits.assign(mLayout->UnitCount(), Unit());
    mCache.Reset(mLayout->LeafCount());
    mChunks.assign(mLayout->ChunkCount(), Chunk());
    for (int unit = 0; unit < mLayout->UnitCount(); unit++) {
        mChunks[mLayout->UnitChunk(unit)].units.push_back(unit);
    }
}

int DexelRenderer::Update(DexelStock& stock, ThreadPool& pool, double budgetMs, bool& pending)
{
    if (!mLayout) {
        pending = false;
        return 0;
    }
    // a changed leaf changes the polygons of its unit and of the units of its neighbors
    for (int leaf : stock.TakeDirtyLeaves()) {
        mCache.Invalidate(leaf);
        for (int unit : mLayout->AffectedUnits(leaf)) {
            mUnits[unit].dirty = true;
        }
    }
    std::vector<int> dirty;
    for (int i = 0; i < (int)mUnits.size(); i++) {
        if (mUnits[i].dirty) {
            dirty.push_back(i);
        }
    }
    if (dirty.empty()) {
        pending = false;
        return 0;
    }

    // batches of units until the time is used up: the surface of the leaves they need, then
    // their polygons
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::duration<double, std::milli>(budgetMs);
    std::vector<uint8_t> built(dirty.size(), 0);
    const size_t batch = 4 * (size_t)pool.Size();
    std::vector<int> leaves;
    for (size_t first = 0; first < dirty.size() && std::chrono::steady_clock::now() < deadline;
         first += batch) {
        const size_t last = std::min(dirty.size(), first + batch);
        leaves.clear();
        for (size_t i = first; i < last; i++) {
            const auto& needed = mLayout->UnitDependencies(dirty[i]);
            leaves.insert(leaves.end(), needed.begin(), needed.end());
        }
        mCache.Update(stock, leaves, pool);
        std::atomic<size_t> next {first};
        pool.Run([&](int) {
            while (true) {
                const size_t i = next++;
                if (i >= last) {
                    break;
                }
                BuildDexelMesh(stock, mCache, mLayout->UnitLeaves(dirty[i]), mUnits[dirty[i]].mesh);
                built[i] = 1;
            }
        });
    }
    int count = 0;
    for (size_t i = 0; i < dirty.size(); i++) {
        if (built[i]) {
            mUnits[dirty[i]].dirty = false;
            mChunks[mLayout->UnitChunk(dirty[i])].changed = true;
            count++;
        }
    }
    for (Chunk& chunk : mChunks) {
        if (chunk.changed) {
            Upload(chunk);
        }
    }
    pending = count < (int)dirty.size();
    return count;
}

void DexelRenderer::Upload(Chunk& chunk)
{
    chunk.changed = false;
    std::vector<DexelVertex> vertices;
    std::vector<uint32_t> indices;
    size_t nv = 0, ni = 0;
    for (int u : chunk.units) {
        nv += mUnits[u].mesh.vertices.size();
        ni += mUnits[u].mesh.stockIndices.size() + mUnits[u].mesh.cutIndices.size();
    }
    vertices.reserve(nv);
    indices.reserve(ni);
    // stock faces first, then cut faces
    for (int pass = 0; pass < 2; pass++) {
        size_t base = 0;
        for (int u : chunk.units) {
            const DexelMesh& mesh = mUnits[u].mesh;
            const auto& list = pass == 0 ? mesh.stockIndices : mesh.cutIndices;
            for (uint32_t index : list) {
                indices.push_back((uint32_t)(base + index));
            }
            base += mesh.vertices.size();
        }
        if (pass == 0) {
            chunk.stockCount = (int)indices.size();
        }
    }
    chunk.cutCount = (int)indices.size() - chunk.stockCount;
    for (int u : chunk.units) {
        const auto& v = mUnits[u].mesh.vertices;
        vertices.insert(vertices.end(), v.begin(), v.end());
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
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(DexelVertex), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, chunk.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32_t), indices.data(), GL_STATIC_DRAW);
}

void DexelRenderer::Draw(const Chunk& chunk, bool cut) const
{
    const int count = cut ? chunk.cutCount : chunk.stockCount;
    if (count == 0 || chunk.vbo == 0) {
        return;
    }
    glBindBuffer(GL_ARRAY_BUFFER, chunk.vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(DexelVertex), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_BYTE, GL_TRUE, sizeof(DexelVertex), (void*)offsetof(DexelVertex, nx));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, chunk.ibo);
    const size_t offset = cut ? chunk.stockCount * sizeof(uint32_t) : 0;
    glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, (void*)offset);
}

void DexelRenderer::Render(SimDisplay& display, const float stockColor[3], const float cutColor[3])
{
    const vec3 colors[2] = {
        {stockColor[0], stockColor[1], stockColor[2]},
        {cutColor[0], cutColor[1], cutColor[2]},
    };
    for (int pass = 0; pass < 2; pass++) {
        display.StartGeometryPass(colors[pass], false);
        CurrentShader->UpdateModelMat(identityMat, identityMat);
        for (const Chunk& chunk : mChunks) {
            Draw(chunk, pass == 1);
        }
    }
}

long DexelRenderer::TriangleCount() const
{
    long count = 0;
    for (const Chunk& chunk : mChunks) {
        count += (chunk.stockCount + chunk.cutCount) / 3;
    }
    return count;
}

}  // namespace CAMSimulator
