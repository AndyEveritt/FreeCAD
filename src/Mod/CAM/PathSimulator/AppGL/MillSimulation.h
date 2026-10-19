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
#include "MachineKinematics.h"
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
#include <utility>
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

// A problem found on the toolpath, where it is on the timeline: the step of its start and its command
struct SimEvent
{
    Material::Event event;
    int step = 0;
    int command = -1;
};

struct MillSimulationState
{
    int mCurStep = 0;
    int mNTotalSteps = 0;
    int mPathStep = -1;
    int mSubStep = 0;
    int mNPathSteps = 0;
    int mSimSpeed = 10;
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
    // arcs: of the profile, see Material::Tool. flutes: their length from the tip, 0 if unknown.
    // rates: for the time of the moves. cutting: the heights from the tip between which the tool
    // cuts, for its display; (0, flutes) if the second is below 0.
    void AddTool(
        const std::vector<float>& toolProfile,
        int toolid,
        float diameter,
        const std::vector<Material::ProfileArc>& arcs = {},
        double flutes = 0,
        const ToolRates& rates = {},
        std::pair<double, double> cutting = {0, -1}
    );
    // The rapid rate of tools without one, mm/s
    void SetRapidRate(double rate);
    // The machine of the job: the limits of its axes are checked (Event::MachineLimit), rapid
    // moves of tools without a rapid rate go at the velocities of its axes
    void SetMachine(const MachineKinematics& machine);

    // The timeline is in the time of the machine: a step a millisecond, at least one a motion.
    // The stencil renderer has its own steps, by the length of the motions.
    static constexpr double StepsPerSecond = 1000;
    // The time of the machine at the current step and in all, s
    double CurrentSeconds() const;
    double TotalSeconds() const;
    // The time of each operation, s
    std::vector<double> OperationSeconds() const;
    bool ToolExists(int toolid);
    void RenderSimulation();
    void RenderTool();
    void RenderToolShape(const EndMill* tool, const mat4x4& tmat, const mat4x4& nmat);
    void RenderPath();
    void RenderBaseShape();
    void RenderOrigin();
    void Render();
    void ProcessSim(const clock::duration& elapsed);
    void SimNext(const clock::duration& elapsed);

    bool LoadGCodeFile(const char* fileName);
    bool AddGcodeLine(const char* line, int source = -1);
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
    // The toolpath command (MillMotion::source) in progress at the current step, -1 if none
    int CurrentCommand() const;
    // Go to the start of the first command from this index on that moves the tool
    void GoToCommand(int source);
    // Go to the start of the next command that moves the tool (direction > 0), or of the command
    // in progress, or the previous one if already there (direction < 0)
    void StepCommand(int direction);
    // The triangles of the stock shown, a closed mesh, nothing before the simulation shows one
    void StockTriangles(std::vector<float>& xyz, std::vector<uint32_t>& triangles) const;
    // The problems found on the toolpath so far, in the order of the path
    std::vector<SimEvent> GetEvents() const;
    int EventCount() const;
    // Go to the start of an event, or of the next one after the current step (direction > 0) or
    // the last one before it (direction < 0)
    void GoToEvent(int index);
    void StepEvent(int direction);
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
    // The origin of the job (of the workpiece coordinates) and its axes
    void SetOriginVisible(bool b);
    // The tool at its position, the part that cuts in its own color
    void SetToolVisible(bool b);
    // A section of the stock and the model across an axis (0 X, 1 Y, 2 Z, -1 for none), at a
    // fraction of the box of the stock along it: the side below is kept, the side above with flip
    void SetSection(int axis, float position, bool flip);

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
    // Inspect 4: a click picks a point of a surface, two make a distance; the wall behind the
    // first one, through the stock, is measured too
    bool Measures() const
    {
        return mInspect == 4;
    }
    struct Measure
    {
        int points = 0;  // 0, 1 or 2
        float a[3] = {0, 0, 0};
        float b[3] = {0, 0, 0};
        double wall = -1;  // the thickness of the stock behind a, negative for none
    };
    const Measure& GetMeasure() const
    {
        return mMeasure;
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
    // The steps of the segments, of the operations and in all, for the renderer in use
    void ApplyTimeline();
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
    std::vector<int> mSegmentFirstStep;        // the first step of each segment
    std::map<int, std::vector<Material::ProfileArc>> mToolArcs;  // by tool id
    std::map<int, double> mToolFlutes;                           // by tool id
    std::map<int, ToolRates> mToolRates;                         // by tool id
    double mRapidRate = 5000.0 / 60;
    MachineKinematics mMachine;
    std::vector<Material::Event> mMachineEvents;
    // of the segments: the time of the machine, its start, and the steps of the stencil renderer
    std::vector<double> mSegmentSeconds;
    std::vector<double> mSegmentStartSeconds;
    std::vector<int> mSegmentStencilSteps;
    std::vector<int> mSegmentStencilSweepSteps;
    std::vector<int> mSegmentSteps;  // on the timeline
    std::vector<SimOperation> mOperations;
    StockColoring mColoring;
    bool mThinWalls = false;
    bool mDeviation = false;
    int mInspect = 0;
    Measure mMeasure;
    unsigned int mMeasureVbo = 0;
    void RenderMeasure();
    void ApplyInspect();
    std::shared_ptr<const Material::ModelDistance> mDeviationModel;
    float mStockOpacity = 1;
    float mBaseOpacity = 1;
    float mThinThreshold = 0.5f;
    bool mOriginVisible = true;
    bool mToolVisible = true;
    int mSectionAxis = -1;
    float mSectionPosition = 0.5f;
    bool mSectionFlip = false;
    void ApplySection();
    // the box of the stock
    void StockBox(float lo[3], float hi[3]) const;
    unsigned int mOriginVbo = 0;
    float mOriginLength = 0;  // of the axes in mOriginVbo

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
    vec3 toolColor = {0.5f, 0.4f, 0.3f};  // a tool whose cutting part is unknown
    vec3 fluteColor = {0.9f, 0.55f, 0.15f};  // the cutting length of the tool
    vec3 shankColor = {0.55f, 0.56f, 0.6f};  // the rest of the tool
    vec3 baseShapeColor = {0.7f, 0.6f, 0.5f};

    clock::duration mTotalElapsed;
};

}  // namespace CAMSimulator
