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

#include "DexelMesher.h"

#include <vector>

namespace CAMSimulator
{

class SimDisplay;
class ThreadPool;

// Renders a DexelStock as a mesh. The mesh is built per mesh unit of the layout, small groups of
// leaves that are rebuilt when they change, and drawn per chunk of units, to keep the number of
// draw calls low.
class DexelRenderer
{
public:
    ~DexelRenderer();

    // Start with a new stock, all units are built by the next Update
    void Reset(const DexelStock& stock);
    // Rebuild the units of the dirty leaves of the stock, for about budgetMs. Returns the number
    // of rebuilt units, 'pending' tells if units are left to rebuild.
    int Update(DexelStock& stock, ThreadPool& pool, double budgetMs, bool& pending);
    void Render(SimDisplay& display, const float stockColor[3], const float cutColor[3]);
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
        std::vector<int> units;
    };
    struct Unit
    {
        DexelMesh mesh;
        bool dirty = true;
    };

    void Draw(const Chunk& chunk, bool cut) const;
    void Upload(Chunk& chunk);

    std::shared_ptr<const DexelLayout> mLayout;
    DexelMeshCache mCache;
    std::vector<Unit> mUnits;
    std::vector<Chunk> mChunks;
};

}  // namespace CAMSimulator
