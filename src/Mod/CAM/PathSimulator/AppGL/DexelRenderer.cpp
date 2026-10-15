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
    mLayout.reset();
}

void DexelRenderer::Reset(const DexelStock& stock)
{
    FreeResources();
    mLayout = stock.LayoutRef();
    mUnits.Reset(stock);
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
    const std::vector<int> updated = mUnits.Update(stock, pool, budgetMs);
    for (int unit : updated) {
        mChunks[mLayout->UnitChunk(unit)].changed = true;
    }
    for (Chunk& chunk : mChunks) {
        if (chunk.changed) {
            Upload(chunk);
        }
    }
    pending = mUnits.Pending();
    return (int)updated.size();
}

void DexelRenderer::Upload(Chunk& chunk)
{
    chunk.changed = false;
    std::vector<DexelVertex> vertices;
    std::vector<uint32_t> indices;
    size_t nv = 0, ni = 0;
    for (int u : chunk.units) {
        const DexelMesh& mesh = mUnits.Mesh(u);
        nv += mesh.vertices.size();
        ni += mesh.stockIndices.size() + mesh.cutIndices.size();
    }
    vertices.reserve(nv);
    indices.reserve(ni);
    // stock faces first, then cut faces
    for (int pass = 0; pass < 2; pass++) {
        size_t base = 0;
        for (int u : chunk.units) {
            const DexelMesh& mesh = mUnits.Mesh(u);
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
        const auto& v = mUnits.Mesh(u).vertices;
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
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_UNSIGNED_BYTE, GL_FALSE, sizeof(DexelVertex), (void*)offsetof(DexelVertex, operation));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(DexelVertex), (void*)offsetof(DexelVertex, thickness));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, chunk.ibo);
    const size_t offset = cut ? chunk.stockCount * sizeof(uint32_t) : 0;
    glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, (void*)offset);
    // other meshes only have positions and normals
    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(3);
}

void DexelRenderer::Render(
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

long DexelRenderer::TriangleCount() const
{
    long count = 0;
    for (const Chunk& chunk : mChunks) {
        count += (chunk.stockCount + chunk.cutCount) / 3;
    }
    return count;
}

}  // namespace CAMSimulator
