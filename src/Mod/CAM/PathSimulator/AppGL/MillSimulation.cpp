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

#include "MillSimulation.h"

#include "GlUtils.h"
#include <algorithm>
#include <iostream>

// include this last as the defines can mess up other includes
#include "OpenGlWrapper.h"

using namespace std::literals;

#define DRAG_ZOOM_FACTOR 10

namespace CAMSimulator
{

MillSimulation::MillSimulation()
{}

MillSimulation::~MillSimulation()
{
    Clear();
}

void MillSimulation::ClearMillPathSegments()
{
    for (unsigned int i = 0; i < MillPathSegments.size(); i++) {
        delete MillPathSegments[i];
    }
    MillPathSegments.clear();
}

void MillSimulation::Clear()
{
    mCodeParser.Clear();

    ClearMillPathSegments();

    for (unsigned int i = 0; i < mToolTable.size(); i++) {
        delete mToolTable[i];
    }
    mToolTable.clear();

    mStockObject.Clear();
    simDisplay.CleanGL();

    mCurStep = 0;
    mPathStep = -1;
    mNTotalSteps = 0;

    simulationInitiated = false;
}

void MillSimulation::InitSimulation(float quality, float maxStockDimension)
{
    ClearMillPathSegments();
    millPathLine.Clear();
    // mViewSSAO = guiDisplay.IsChecked(eGuiItemAmbientOclusion);

    // gDestPos = curMillOperation->startPos;
    mCurStep = 0;
    mPathStep = -1;
    mNTotalSteps = 0;
    mSimPlaying = false;
    mSimSpeed = 1;
    mTotalElapsed = 0s;

    MillPathSegment::SetQuality(quality, maxStockDimension);

    int nOperations = (int)mCodeParser.Operations.size();
    int segId = 0;

    MillMotion prevMotion = !mCodeParser.Operations.empty() ? mCodeParser.Operations.front()
                                                            : MillMotion();

    MillPathPosition mpPos;
    mpPos.X = prevMotion.x;
    mpPos.Y = prevMotion.y;
    mpPos.Z = prevMotion.z;
    mpPos.SegmentId = segId++;
    millPathLine.MillPathPointsBuffer.push_back(mpPos);

    for (int i = 1; i < nOperations; i++) {
        const MillMotion curMotion = mCodeParser.Operations[i];
        const EndMill* tool = GetTool(curMotion.tool);
        if (tool != nullptr) {
            auto segment = new MillPathSegment(*tool, prevMotion, curMotion);
            segment->indexInArray = i;
            segment->segmentIndex = segId++;
            mNTotalSteps += segment->numSimSteps;
            MillPathSegments.push_back(segment);
            segment->AppendPathPoints(millPathLine.MillPathPointsBuffer);
        }

        prevMotion = curMotion;
    }

    assert(mNTotalSteps >= 0);

    mNPathSteps = (int)MillPathSegments.size();
    millPathLine.GenerateModel();

    InitDisplay(quality);
    simDisplay.simCacheValid = false;

    simulationInitiated = true;
}

EndMill* MillSimulation::GetTool(int toolId)
{
    for (unsigned int i = 0; i < mToolTable.size(); i++) {
        if (mToolTable[i]->toolId == toolId) {
            return mToolTable[i];
        }
    }
    return nullptr;
}

void MillSimulation::RemoveTool(const int toolId)
{
    EndMill* tool = GetTool(toolId);
    if (tool == nullptr) {
        return;
    }

    if (const auto it = std::ranges::find(mToolTable, tool); it != mToolTable.end()) {
        mToolTable.erase(it);
    }
    delete tool;
}


void MillSimulation::AddTool(EndMill* tool)
{
    // if we have another tool with same id, remove it
    RemoveTool(tool->toolId);
    mToolTable.push_back(tool);
}

void MillSimulation::AddTool(const std::vector<float>& toolProfile, int toolid, float diameter)
{
    // if we have another tool with same id, remove it
    RemoveTool(toolid);
    EndMill* tool = new EndMill(toolProfile, toolid, diameter);
    mToolTable.push_back(tool);
}

bool MillSimulation::ToolExists(int toolid)
{
    return GetTool(toolid) != nullptr;
}

void MillSimulation::GlsimStart()
{
    glDisable(GL_BLEND);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glEnable(GL_STENCIL_TEST);
}

void MillSimulation::GlsimToolStep1(void)
{
    glCullFace(GL_BACK);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_FALSE);
    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilOp(GL_ZERO, GL_ZERO, GL_REPLACE);
}

void MillSimulation::GlsimToolStep2(void)
{
    glCullFace(GL_FRONT);
    glDepthFunc(GL_GREATER);
    glDepthMask(GL_TRUE);
    glStencilFunc(GL_EQUAL, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
}

void MillSimulation::GlsimClipBack(void)
{
    glCullFace(GL_FRONT);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_FALSE);
    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilOp(GL_REPLACE, GL_REPLACE, GL_ZERO);
}

void MillSimulation::GlsimRenderStock(void)
{
    glCullFace(GL_BACK);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthFunc(GL_EQUAL);
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_EQUAL, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
}

void MillSimulation::GlsimRenderTools(void)
{
    glCullFace(GL_FRONT);
}

void MillSimulation::GlsimEnd(void)
{
    glCullFace(GL_BACK);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);

    glDisable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 1, 0xFF);
}

// Get the range of steps of segment iSeg up to the current simulation position. If 'newOnly' is
// set, steps already contained in the simulation cache are excluded. Returns false if the range
// is empty.
bool MillSimulation::GetSegmentStepRange(int iSeg, bool newOnly, int& first, int& last) const
{
    const MillPathSegment* p = MillPathSegments.at(iSeg);
    last = iSeg == mPathStep ? mSubStep : p->numSimSteps;
    first = p->isMultyPart ? 1 : last;
    if (newOnly && iSeg == mCachedPathStep) {
        if (last <= mCachedSubStep) {
            return false;
        }
        if (p->isMultyPart) {
            first = mCachedSubStep + 1;
        }
    }
    return first <= last;
}

void MillSimulation::renderSegmentForward(int iSeg, bool newOnly)
{
    int first, last;
    if (!GetSegmentStepRange(iSeg, newOnly, first, last)) {
        return;
    }
    MillPathSegment* p = MillPathSegments.at(iSeg);
    for (int i = first; i <= last; i++) {
        GlsimToolStep1();
        p->render(i);
        GlsimToolStep2();
        p->render(i);
    }
}

void MillSimulation::renderSegmentReversed(int iSeg, bool newOnly)
{
    int first, last;
    if (!GetSegmentStepRange(iSeg, newOnly, first, last)) {
        return;
    }
    MillPathSegment* p = MillPathSegments.at(iSeg);
    for (int i = last; i >= first; i--) {
        GlsimToolStep1();
        p->render(i);
        GlsimToolStep2();
        p->render(i);
    }
}

void MillSimulation::CalcSegmentPositions()
{
    mSubStep = mCurStep;
    for (mPathStep = 0; mPathStep < mNPathSteps; mPathStep++) {
        MillPathSegment* p = MillPathSegments[mPathStep];
        if (mSubStep < p->numSimSteps) {
            break;
        }
        mSubStep -= p->numSimSteps;
    }
    if (mPathStep >= mNPathSteps) {
        mPathStep = mNPathSteps - 1;
        mSubStep = MillPathSegments[mPathStep]->numSimSteps;
    }
    else {
        mSubStep++;
    }
}

void MillSimulation::RenderSimulation()
{
    const bool visible = (mViewItems & VIEWITEM_SIMULATION) != 0;
    if (!visible || !simDisplay.CanCacheSimulation()) {
        // render directly to the display frame buffer
        simDisplay.PrepareFrameBuffer();
        if (visible) {
            RenderStockCuts(false);
        }
        return;
    }

    const bool cacheValid = simDisplay.simCacheValid;
    if (!cacheValid) {
        // checkpoints are invalidated along with the cache
        mCheckpoints.clear();
    }

    // An incremental update only subtracts the new steps from the cached result, so it may miss
    // overlaps that the full render resolves by re-applying all steps in several orders. It is
    // therefore only used while playing or scrubbing, and a full render is done once the
    // simulation rests.
    const bool interactive = mSimPlaying || mScrubbing;

    const bool samePosition = mPathStep == mCachedPathStep && mSubStep == mCachedSubStep;
    if (cacheValid && samePosition && (interactive || !mCacheIncremental)) {
        // nothing changed, reuse the cached result
        simDisplay.EndSimulationPass();
        return;
    }

    bool incremental = false;
    if (cacheValid && interactive) {
        const bool forward = mPathStep > mCachedPathStep
            || (mPathStep == mCachedPathStep && mSubStep >= mCachedSubStep);
        incremental = forward;

        // continue from the latest checkpoint before the current position, unless the cache is
        // already closer
        int bestSlot = -1;
        int bestStep = forward ? mCachedCurStep : -1;
        for (int i = 0; i < (int)mCheckpoints.size(); i++) {
            const int step = mCheckpoints[i].curStep;
            if (step >= 0 && step <= mCurStep && step > bestStep) {
                bestSlot = i;
                bestStep = step;
            }
        }
        if (bestSlot >= 0) {
            simDisplay.RestoreSimulationCheckpoint(bestSlot);
            mCachedPathStep = mCheckpoints[bestSlot].pathStep;
            mCachedSubStep = mCheckpoints[bestSlot].subStep;
            mCachedCurStep = mCheckpoints[bestSlot].curStep;
            incremental = true;
        }
    }

    simDisplay.StartSimulationPass(!incremental);
    RenderStockCuts(incremental);
    mCachedPathStep = mPathStep;
    mCachedSubStep = mSubStep;
    mCachedCurStep = mCurStep;
    mCacheIncremental = incremental;

    UpdateSimulationCheckpoint();

    simDisplay.EndSimulationPass();
}

// Keep a copy of the simulation cache at evenly spaced positions, so moving backwards in the
// simulation can continue from a nearby checkpoint instead of rendering all steps again.
void MillSimulation::UpdateSimulationCheckpoint()
{
    const int nCheckpoints = simDisplay.MaxSimulationCheckpoints();
    if (nCheckpoints <= 0 || mNTotalSteps <= 0) {
        return;
    }
    mCheckpoints.resize(nCheckpoints);

    // each slot covers a range of steps, prefer the earliest position within that range
    const int slot
        = std::min((int)((long long)mCurStep * nCheckpoints / mNTotalSteps), nCheckpoints - 1);
    SimPosition& checkpoint = mCheckpoints[slot];
    if (checkpoint.curStep >= 0 && checkpoint.curStep <= mCurStep) {
        return;
    }

    if (simDisplay.SaveSimulationCheckpoint(slot)) {
        checkpoint = {mCurStep, mPathStep, mSubStep};
    }
}

// Render the stock with all cuts up to the current simulation position. If 'incremental' is set,
// only the cuts that are not yet part of the simulation cache are added to it.
void MillSimulation::RenderStockCuts(bool incremental)
{
    const int firstSeg = incremental ? std::max(mCachedPathStep, 0) : 0;

    simDisplay.StartDepthPass();

    GlsimStart();
    if (!incremental) {
        mStockObject.render();
    }

    GlsimToolStep2();

    for (int i = firstSeg; i <= mPathStep; i++) {
        renderSegmentForward(i, incremental);
    }

    for (int i = mPathStep; i >= firstSeg; i--) {
        renderSegmentForward(i, incremental);
    }

    for (int i = firstSeg; i < mPathStep; i++) {
        renderSegmentReversed(i, incremental);
    }

    for (int i = mPathStep; i >= firstSeg; i--) {
        renderSegmentReversed(i, incremental);
    }

    GlsimClipBack();
    mStockObject.render();

    // start coloring
    if (incremental) {
        // the stock surface only changed where new cuts were made. Those are either colored by
        // the cuts below, or are cut through the stock and need to be cleared.
        simDisplay.ClearGeometryOutsideStencil();
        GlsimRenderStock();
    }
    else {
        simDisplay.StartGeometryPass(stockColor, false);
        GlsimRenderStock();
        mStockObject.render();
    }

    // render cuts (back faces of tools)
    simDisplay.StartGeometryPass(cutColor, true);
    GlsimRenderTools();
    for (int i = firstSeg; i <= mPathStep; i++) {
        int first, last;
        if (!GetSegmentStepRange(i, incremental, first, last)) {
            continue;
        }
        for (int j = first; j <= last; j++) {
            MillPathSegments.at(i)->render(j);
        }
    }

    GlsimEnd();
}

void MillSimulation::RenderTool()
{
    if (mPathStep < 0) {
        return;
    }

    MillPathSegment* p = MillPathSegments.at(mPathStep);
    vec3 toolPos;
    p->GetHeadPosition(toolPos);
    mat4x4 tmat;
    mat4x4_translate(tmat, toolPos[0], toolPos[1], toolPos[2]);
    // mat4x4_translate(tmat, toolPos.x, toolPos.y, toolPos.z);
    simDisplay.StartGeometryPass(toolColor, false);
    p->endmill->toolShape.Render(tmat, identityMat);
}

void MillSimulation::RenderPath()
{
    if (!mViewPath) {
        return;
    }
    simDisplay.SetupLinePathPass(mPathStep, false);
    millPathLine.Render();
    simDisplay.SetupLinePathPass(mPathStep, true);
    millPathLine.Render();
    glDepthMask(GL_TRUE);
}

void MillSimulation::RenderBaseShape()
{
    if ((mViewItems & VIEWITEM_BASE_SHAPE) == 0) {
        return;
    }
    simDisplay.StartDepthPass();
    glPolygonOffset(0, -2);
    glEnable(GL_POLYGON_OFFSET_FILL);
    simDisplay.StartGeometryPass(baseShapeColor, false);
    mBaseShape.render();
    glDisable(GL_POLYGON_OFFSET_FILL);
}

void MillSimulation::Render()
{
    if (!simulationInitiated) {
        return;
    }

    // set background
    glClearColor(bgndColor[0], bgndColor[1], bgndColor[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    // render the simulation offscreen in an FBO

    if (simDisplay.updateDisplay) {
        RenderSimulation();
        RenderTool();
        RenderBaseShape();
        RenderPath();
        simDisplay.updateDisplay = false;
        simDisplay.RenderResult(true, mViewSSAO);
    }
    else {
        simDisplay.RenderResult(false, mViewSSAO);
    }

    /*   if (mDebug > 0) {
           mat4x4 test;
           mat4x4_identity(test);
           mat4x4_translate_in_place(test, 20, 20, 3);
           mat4x4_rotate_Z(test, test, 30.f * 3.14f / 180.f);
           int dpos = mNPathSteps - mDebug2;
           MillPathSegment* p = MillPathSegments.at(dpos);
           if (mDebug > p->numSimSteps) {
               mDebug = 1;
           }
           p->render(mDebug);
       }*/

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void MillSimulation::ProcessSim(const clock::duration& elapsed)
{
    SimNext(elapsed);
    Render();
}

void MillSimulation::SimNext(const clock::duration& elapsed)
{
    // calculate number of steps based on elapsed time

    int numSteps = 0;

    if (mSimPlaying) {
        mTotalElapsed += elapsed;

        const float seconds
            = std::chrono::duration_cast<std::chrono::duration<float>>(mTotalElapsed).count();

        const float secondsToSteps = mSimSpeed * 60;
        numSteps = seconds * secondsToSteps;

        const std::chrono::duration<float> processed {(float)numSteps / secondsToSteps};
        mTotalElapsed -= std::chrono::duration_cast<clock::duration>(processed);
    }
    else if (mSingleStep) {
        numSteps = 1;

        mTotalElapsed = 0s;
        mSingleStep = false;
    }
    else {
        return;
    }

    // advance simulation

    const int oldStep = mCurStep;

    mCurStep += numSteps;
    mCurStep = std::clamp(mCurStep, 0, mNTotalSteps);
    if (mCurStep == mNTotalSteps) {
        mSimPlaying = false;
    }

    if (mCurStep != oldStep) {
        CalcSegmentPositions();
        simDisplay.updateDisplay = true;
    }
}

void MillSimulation::InitDisplay(float quality)
{
    // generate tools
    for (unsigned int i = 0; i < mToolTable.size(); i++) {
        mToolTable[i]->GenerateDisplayLists(quality);
    }

    // Make sure the next call to UpdateWindowScale will not return early.
    mWidth = -1;
    mHeight = -1;

    // init 3d display
    simDisplay.InitGL();
}

void MillSimulation::SetBoxStock(float x, float y, float z, float l, float w, float h)
{
    mStockObject.GenerateBoxStock(x, y, z, l, w, h);
    simDisplay.ScaleViewToStock(&mStockObject);
    simDisplay.simCacheValid = false;
}

void MillSimulation::SetArbitraryStock(
    const std::vector<Vertex>& verts,
    const std::vector<GLushort>& indices
)
{
    mStockObject.GenerateSolid(verts, indices);
    simDisplay.ScaleViewToStock(&mStockObject);
    simDisplay.simCacheValid = false;
}

void MillSimulation::SetStockVisible(bool b)
{
    if (b == IsStockVisible()) {
        return;
    }

    mViewItems ^= VIEWITEM_SIMULATION;
    simDisplay.simCacheValid = false;
    simDisplay.updateDisplay = true;
}

bool MillSimulation::IsStockVisible() const
{
    return mViewItems & VIEWITEM_SIMULATION;
}

void MillSimulation::SetBaseObject(const std::vector<Vertex>& verts, const std::vector<GLushort>& indices)
{
    mBaseShape.GenerateSolid(verts, indices);
}

void MillSimulation::SetBaseVisible(bool b)
{
    if (b == IsBaseVisible()) {
        return;
    }

    mViewItems ^= VIEWITEM_BASE_SHAPE;
    simDisplay.updateDisplay = true;
}

bool MillSimulation::IsBaseVisible() const
{
    return mViewItems & VIEWITEM_BASE_SHAPE;
}

void MillSimulation::UpdateWindowScale(int width, int height)
{
    if (width == mWidth && height == mHeight) {
        return;
    }

    mWidth = width;
    mHeight = height;

    simDisplay.UpdateWindowScale(width, height);
}

void MillSimulation::SetPathVisible(bool b)
{
    if (b == mViewPath) {
        return;
    }

    mViewPath = b;
    simDisplay.updateDisplay = true;
}

void MillSimulation::EnableSsao(bool b)
{
    if (b == mViewSSAO) {
        return;
    }

    mViewSSAO = b;
    simDisplay.updateDisplay = true;
}

void MillSimulation::UpdateCamera(const SoCamera& camera)
{
    simDisplay.UpdateCamera(camera);
}

bool MillSimulation::LoadGCodeFile(const char* fileName)
{
    if (mCodeParser.Parse(fileName)) {
        std::cout << "GCode file loaded successfully" << std::endl;
        return true;
    }
    return false;
}

bool MillSimulation::AddGcodeLine(const char* line)
{
    return mCodeParser.AddLine(line);
}

void MillSimulation::SetPlaying(bool b)
{
    if (b == mSimPlaying) {
        return;
    }

    mSimPlaying = b;
    simDisplay.updateDisplay = true;
}

void MillSimulation::SingleStep()
{
    if (!mSimPlaying && mSingleStep) {
        return;
    }

    mSimPlaying = false;
    mSingleStep = true;
    simDisplay.updateDisplay = true;
}

void MillSimulation::SetSpeed(int s)
{
    mSimSpeed = s;
}

void MillSimulation::SetSimulationStage(float stage)
{
    const int newStep = (int)((float)mNTotalSteps * stage);
    if (newStep == mCurStep) {
        return;
    }

    mCurStep = newStep;
    mSingleStep = true;
    CalcSegmentPositions();

    simDisplay.updateDisplay = true;
}

// True while the displayed simulation does not yet match the current simulation position, for
// renderers that spread the work over several frames.
bool MillSimulation::IsRenderPending() const
{
    return false;
}

void MillSimulation::SetScrubbing(bool b)
{
    if (b == mScrubbing) {
        return;
    }

    mScrubbing = b;
    simDisplay.updateDisplay = true;
}

void MillSimulation::SetState(const MillSimulationState& state)
{
    SetPlaying(state.mSimPlaying);
    if (state.mSingleStep) {
        SingleStep();
    }

    const float stage = (float)state.mCurStep / state.mNTotalSteps;
    SetSimulationStage(stage);

    SetSpeed(state.mSimSpeed);

    if (mViewItems != state.mViewItems) {
        simDisplay.simCacheValid = false;
    }
    mViewItems = state.mViewItems;
    mViewPath = state.mViewPath;
    mViewSSAO = state.mViewSSAO;
}

const MillSimulationState& MillSimulation::GetState() const
{
    return *this;
}

void MillSimulation::SetBackgroundColor(const vec3& c)
{
    bgndColor[0] = c[0];
    bgndColor[1] = c[1];
    bgndColor[2] = c[2];
}

void MillSimulation::SetPathColor(const vec3& normal, const vec3& rapid)
{
    simDisplay.SetPathColor(normal, rapid);
}


}  // namespace CAMSimulator
