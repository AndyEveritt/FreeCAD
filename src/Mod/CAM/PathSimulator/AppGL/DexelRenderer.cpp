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
#include <chrono>
#include <cstddef>

// include this last as the defines can mess up other includes
#include "OpenGlWrapper.h"

namespace CAMSimulator
{

constexpr int ChunkSize = 64;  // cells, a multiple of DexelTileSize

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
    mChunksX = mChunksY = 0;
}

void DexelRenderer::Reset(const DexelStock& stock)
{
    FreeResources();
    mNX = stock.NX();
    mNY = stock.NY();
    mTilesX = stock.TilesX();
    mChunksX = (mNX + ChunkSize - 1) / ChunkSize;
    mChunksY = (mNY + ChunkSize - 1) / ChunkSize;
    mChunks.assign((size_t)mChunksX * mChunksY, Chunk());
}

int DexelRenderer::Update(
    DexelStock& stock,
    const DexelStock& initial,
    ThreadPool& pool,
    double budgetMs,
    bool& pending
)
{
    // a changed tile changes the chunk containing it, and the faces of neighboring chunks along
    // its border
    constexpr int tilesPerChunk = ChunkSize / DexelTileSize;
    for (int tile : stock.TakeDirtyTiles()) {
        const int tx = tile % mTilesX, ty = tile / mTilesX;
        for (int ny = ty - 1; ny <= ty + 1; ny++) {
            for (int nx = tx - 1; nx <= tx + 1; nx++) {
                const int cx = nx / tilesPerChunk, cy = ny / tilesPerChunk;
                if (nx >= 0 && ny >= 0 && cx < mChunksX && cy < mChunksY) {
                    mChunks[cy * mChunksX + cx].dirty = true;
                }
            }
        }
    }

    std::vector<int> dirty;
    for (int i = 0; i < (int)mChunks.size(); i++) {
        if (mChunks[i].dirty) {
            dirty.push_back(i);
        }
    }

    // batches of one chunk per thread, until the time is used up
    const auto start = std::chrono::steady_clock::now();
    const int workers = pool.Size();
    std::vector<DexelMesh> meshes(workers);
    size_t done = 0;
    while (done < dirty.size()) {
        const size_t count = std::min(dirty.size() - done, (size_t)workers);
        pool.Run([&](int worker) {
            if ((size_t)worker >= count) {
                return;
            }
            const int chunk = dirty[done + worker];
            const int cx = chunk % mChunksX, cy = chunk / mChunksX;
            BuildDexelMesh(
                stock,
                initial,
                cx * ChunkSize,
                cy * ChunkSize,
                std::min(mNX, (cx + 1) * ChunkSize),
                std::min(mNY, (cy + 1) * ChunkSize),
                meshes[worker]
            );
        });
        for (size_t i = 0; i < count; i++) {
            Upload(mChunks[dirty[done + i]], meshes[i]);
        }
        done += count;
        const std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now()
            - start;
        if (elapsed.count() > budgetMs) {
            break;
        }
    }
    pending = done < dirty.size();
    return (int)done;
}

void DexelRenderer::Upload(Chunk& chunk, DexelMesh& mesh)
{
    chunk.dirty = false;
    chunk.stockCount = (int)mesh.stockIndices.size();
    chunk.cutCount = (int)mesh.cutIndices.size();
    if (mesh.vertices.empty()) {
        return;
    }
    if (chunk.vbo == 0) {
        glGenBuffers(1, &chunk.vbo);
        glGenBuffers(1, &chunk.ibo);
    }
    mesh.stockIndices.insert(mesh.stockIndices.end(), mesh.cutIndices.begin(), mesh.cutIndices.end());
    glBindBuffer(GL_ARRAY_BUFFER, chunk.vbo);
    glBufferData(
        GL_ARRAY_BUFFER,
        mesh.vertices.size() * sizeof(DexelVertex),
        mesh.vertices.data(),
        GL_STATIC_DRAW
    );
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, chunk.ibo);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        mesh.stockIndices.size() * sizeof(uint32_t),
        mesh.stockIndices.data(),
        GL_STATIC_DRAW
    );
}

void DexelRenderer::Draw(const Chunk& chunk, bool cut) const
{
    const int count = cut ? chunk.cutCount : chunk.stockCount;
    if (count == 0) {
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
            if (chunk.vbo) {
                Draw(chunk, pass == 1);
            }
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
