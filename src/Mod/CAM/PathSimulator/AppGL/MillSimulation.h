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

#include "GCodeParser.h"
#include "MaterialJob.h"
#include "MaterialRenderer.h"
#include "MillPathLine.h"
#include "MillPathSegment.h"
#include "SimDisplay.h"
#include "SolidObject.h"
#include "StockObject.h"
#include "linmath.h"
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#define VIEWITEM_SIMULATION 1
#define VIEWITEM_BASE_SHAPE 2
#define VIEWITEM_MAX 4

namespace CAMSimulator
{

// An operation of the toolpath: its name and its steps, from firstStep to endStep (exclusive)
struct SimOperation
{
    std::string name;
    int firstStep = 0;
    int endStep = 0;
    bool hasColor = false;  // a color of its own, otherwise one of a palette
    vec3 color = {0, 0, 0};
};

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
    // A command of a FreeCAD toolpath, see GCodeParser::AddCommand. source: its index in the
    // toolpath, given to its motions
    bool AddCommand(const std::string& name, const std::map<std::string, double>& params, int source);
    // The G-code lines added from now on belong to a new operation of this name, and of this
    // color if given
    void BeginOperation(const std::string& name, const float* color = nullptr);
    const std::vector<SimOperation>& GetOperations() const
    {
        return mOperations;
    }
    // Index of the operation in progress at a step, -1 if there are none
    int OperationAt(int step) const;
    // Go to the start of the next operation (direction > 0), or of the operation in progress,
    // or the previous one if already there (direction < 0)
    void GoToOperation(int direction);
    // Go to the start of the operation with this index (in the order they were added)
    void GoToOperationIndex(int index);
    // The operation of the stock face at a pixel of the view (from the bottom left), in the next
    // frame: TakePick gives it after that frame, -1 for none (no stock there or uncut stock).
    // 'position' gets the point of the stock there.
    void RequestPick(int x, int y);
    bool TakePick(int& operation, bool& stock, float position[3]);

    void SetPlaying(bool b);
    void SingleStep();
    void SetSpeed(int s);

    void SetSimulationStage(float stage);
    // Go to a step of the simulation, from 0 to the total number of steps
    void SetSimulationStep(int step);
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

    // Cut faces in the color of their operation
    void SetColorByOperation(bool b);
    // Faces in front of material thinner than the threshold in red
    void SetThinWalls(bool enabled, float threshold);
    // Dark lines at the edges of the stock and the other opaque objects
    void SetOutlines(bool enabled);
    // Faces by their distance from the model, within the tolerance green, see StockColoring
    void SetDeviation(bool enabled, float tolerance);
    // The model to measure the distance from, null for none
    void SetDeviationModel(std::shared_ptr<const Material::ModelDistance> model);
    // Lines over the stock to inspect it, see Material::View::Inspect: 0 none, 1 the mesh, 2 the
    // normals, 3 the dexels around the point of the stock clicked
    void SetInspect(int mode);
    bool InspectsDexels() const
    {
        return mInspect == 3;
    }
    // Opacity of the stock and of the base shape (the model), from 0 to 1. The stock of the
    // stencil renderer is always opaque.
    void SetOpacity(float stock, float base);

    void UpdateWindowScale(int width, int height);
    void UpdateCamera(const SoCamera& camera);

    void SetBackgroundColor(const vec3& c);
    void SetPathColor(const vec3& normal, const vec3& rapid);

    // Use the original stencil based renderer instead of the material simulation. It only
    // supports 3 axis toolpaths and gets slow on large toolpaths.
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
    void StartMaterialSimulation();
    Material::Job MaterialJob() const;
    void UpdateMaterialSimulation();
    void RenderMaterialStock();

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

    // material simulation, used unless mStencilRendering
    bool mStencilRendering = false;
    std::unique_ptr<Material::View> mMaterial;
    bool mMaterialNeedsStart = false;
    long mTargetStep = 0;  // step of the material simulation at the tool
    float mQuality = 10;
    std::vector<float> mStockXyz;
    std::vector<uint32_t> mStockTriangles;
    std::vector<Material::Segment> mSegments;  // one per MillPathSegment, tools by id
    std::vector<MillMotion> mSegmentMotions;   // the motion to the end of each segment
    std::vector<SimOperation> mOperations;
    StockColoring mColoring;
    bool mThinWalls = false;
    bool mDeviation = false;
    int mInspect = 0;
    void ApplyInspect();
    std::shared_ptr<const Material::ModelDistance> mDeviationModel;
    float mStockOpacity = 1;
    float mBaseOpacity = 1;
    float mThinThreshold = 0.5f;

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
