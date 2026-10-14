// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2024 Shai Seger <shaise at gmail>                       *
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

#include "DexelRenderer.h"
#include "DexelSimulation.h"
#include "GCodeParser.h"
#include "MillPathLine.h"
#include "MillPathSegment.h"
#include "SimDisplay.h"
#include "SolidObject.h"
#include "StockObject.h"
#include "linmath.h"
#include <chrono>
#include <memory>
#include <vector>

#define VIEWITEM_SIMULATION 1
#define VIEWITEM_BASE_SHAPE 2
#define VIEWITEM_MAX 4

namespace CAMSimulator
{

struct MillSimulationState
{
    int mCurStep = 0;
    int mNTotalSteps = 0;
    int mPathStep = -1;
    int mSubStep = 0;
    int mNPathSteps = 0;
    int mSimSpeed = 1;
    int mViewItems = VIEWITEM_SIMULATION;
    bool mViewPath = false;
    bool mViewSSAO = false;

    bool mSimPlaying = false;
    bool mSingleStep = false;
};

class MillSimulation: private MillSimulationState
{
    typedef std::chrono::steady_clock clock;

public:
    MillSimulation();
    ~MillSimulation();
    void ClearMillPathSegments();
    void Clear();
    void InitSimulation(float quality, float maxStockDimension);
    void AddTool(EndMill* tool);
    void AddTool(const std::vector<float>& toolProfile, int toolid, float diameter);
    bool ToolExists(int toolid);
    void RenderSimulation();
    void RenderTool();
    void RenderPath();
    void RenderBaseShape();
    void Render();
    void ProcessSim(const clock::duration& elapsed);
    void SimNext(const clock::duration& elapsed);

    bool LoadGCodeFile(const char* fileName);
    bool AddGcodeLine(const char* line);

    void SetPlaying(bool b);
    void SingleStep();
    void SetSpeed(int s);

    void SetSimulationStage(float stage);
    bool IsRenderPending() const;
    // True if the next frame changes the display
    bool NeedsFrame() const;
    void SetScrubbing(bool b);
    void SetState(const MillSimulationState& state);
    const MillSimulationState& GetState() const;

    void SetBoxStock(float x, float y, float z, float l, float w, float h);
    void SetArbitraryStock(const std::vector<Vertex>& verts, const std::vector<GLushort>& indices);
    void SetStockVisible(bool b);
    bool IsStockVisible() const;

    void SetBaseObject(const std::vector<Vertex>& verts, const std::vector<GLushort>& indices);
    void SetBaseVisible(bool b);
    bool IsBaseVisible() const;

    void SetPathVisible(bool b);
    void EnableSsao(bool b);

    void UpdateWindowScale(int width, int height);
    void UpdateCamera(const SoCamera& camera);

    void SetBackgroundColor(const vec3& c);
    void SetPathColor(const vec3& normal, const vec3& rapid);

    // Use the original stencil based renderer instead of the dexel simulation. It only supports
    // 3 axis toolpaths and gets slow on large toolpaths.
    void SetStencilRendering(bool b);

protected:
    void InitDisplay(float quality);
    void GlsimStart();
    void GlsimToolStep1(void);
    void GlsimToolStep2(void);
    void GlsimClipBack(void);
    void GlsimRenderStock(void);
    void GlsimRenderTools(void);
    void GlsimEnd(void);
    void RenderStockCuts(bool incremental);
    bool GetSegmentStepRange(int iSeg, bool newOnly, int& first, int& last) const;
    void UpdateSimulationCheckpoint();
    void renderSegmentForward(int iSeg, bool newOnly);
    void renderSegmentReversed(int iSeg, bool newOnly);
    void CalcSegmentPositions();
    EndMill* GetTool(int tool);
    void RemoveTool(int toolId);
    void StartDexelSimulation();
    void UpdateDexelSimulation();
    void RenderDexelStock();

protected:
    bool simulationInitiated = false;

    // position in the simulation, ordered by mCurStep
    struct SimPosition
    {
        int curStep = -1;
        int pathStep = -1;
        int subStep = 0;
    };

    // simulation position held by the simulation cache of simDisplay
    int mCachedPathStep = -1;
    int mCachedSubStep = 0;
    int mCachedCurStep = -1;
    bool mCacheIncremental = false;

    // positions of the simulation checkpoints of simDisplay, curStep < 0 for unused slots
    std::vector<SimPosition> mCheckpoints;

    // true while the user drags the simulation stage slider
    bool mScrubbing = false;

    // dexel simulation, used unless mStencilRendering
    bool mStencilRendering = false;
    std::unique_ptr<DexelSimulation> mDexel;
    DexelRenderer mDexelRenderer;
    bool mDexelNeedsStart = false;
    bool mMeshPending = false;
    double mBudgetScale = 1;  // of the time per frame for simulating and meshing
    std::chrono::steady_clock::time_point mLastFrameStart;
    float mQuality = 10;
    std::vector<float> mStockXyz;
    std::vector<uint32_t> mStockTriangles;
    std::vector<CutSegment> mCutSegments;  // one per MillPathSegment

    // protected:
public:
    std::vector<EndMill*> mToolTable;
    GCodeParser mCodeParser;
    SimDisplay simDisplay;
    MillPathLine millPathLine;
    std::vector<MillPathSegment*> MillPathSegments;

    int mWidth = -1;
    int mHeight = -1;

    StockObject mStockObject;
    SolidObject mBaseShape;

    vec3 bgndColor = {0.1f, 0.2f, 0.3f};
    vec3 stockColor = {0.5f, 0.55f, 0.9f};
    vec3 cutColor = {0.5f, 0.84f, 0.73f};
    vec3 toolColor = {0.5f, 0.4f, 0.3f};
    vec3 baseShapeColor = {0.7f, 0.6f, 0.5f};

    clock::duration mTotalElapsed;
};

}  // namespace CAMSimulator
