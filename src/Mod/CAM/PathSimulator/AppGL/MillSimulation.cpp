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
#include "MaterialJob.h"
#include "ToolpathCheck.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
    mOperations.clear();

    ClearMillPathSegments();

    for (unsigned int i = 0; i < mToolTable.size(); i++) {
        delete mToolTable[i];
    }
    mToolTable.clear();
    mToolArcs.clear();
    mToolFlutes.clear();
    mToolRates.clear();

    mStockObject.Clear();
    GLDELETE_BUFFER(mOriginVbo);
    GLDELETE_BUFFER(mMeasureVbo);
    mMeasure = Measure();
    mMaterial.reset();
    mSegments.clear();
    mSegmentMotions.clear();
    mSegmentFirstStep.clear();
    mSegmentSeconds.clear();
    mSegmentStartSeconds.clear();
    mSegmentStencilSteps.clear();
    mSegmentStencilSweepSteps.clear();
    mSegmentSteps.clear();
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
    mSimSpeed = 10;  // times the time of the machine
    mTotalElapsed = 0s;

    MillPathSegment::SetQuality(quality, maxStockDimension);
    mQuality = quality;
    mSegments.clear();
    mSegmentMotions.clear();
    mSegmentFirstStep.clear();
    mSegmentSeconds.clear();
    mSegmentStartSeconds.clear();
    mSegmentStencilSteps.clear();
    mSegmentStencilSweepSteps.clear();
    mSegmentSteps.clear();

    for (SimOperation& op : mOperations) {
        op.firstStep = op.endStep = -1;
    }
    mCodeParser.FillStart();

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
            Material::Segment cut = MotionSegment(prevMotion, curMotion);
            // arcs outside the XY plane: a line for the stencil renderer, the path along the arc
            const bool otherArc = cut.arc && curMotion.plane != 17;
            MillMotion straight = curMotion;
            if (otherArc) {
                straight.cmd = eMoveLiner;
            }
            auto segment = new MillPathSegment(*tool, prevMotion, straight);
            segment->indexInArray = i;
            segment->segmentIndex = segId++;
            MillPathSegments.push_back(segment);
            const size_t firstPoint = millPathLine.MillPathPointsBuffer.size();
            if (otherArc) {
                const int n = std::max(2, (int)std::ceil(std::abs(cut.sweep) / (M_PI / 32)));
                for (int k = 1; k <= n; k++) {
                    double p[3];
                    cut.PositionAt((double)k / n, p);
                    MillPathPosition point;
                    point.X = (float)p[0];
                    point.Y = (float)p[1];
                    point.Z = (float)p[2];
                    point.SegmentId = segment->segmentIndex;
                    millPathLine.MillPathPointsBuffer.push_back(point);
                }
            }
            else {
                segment->AppendPathPoints(millPathLine.MillPathPointsBuffer);
            }
            for (size_t p = firstPoint; p < millPathLine.MillPathPointsBuffer.size(); p++) {
                millPathLine.MillPathPointsBuffer[p].Rapid = curMotion.rapid ? 1.0f : 0.0f;
            }

            cut.tool = curMotion.tool;  // tool id, replaced by the index when starting
            const auto rates = mToolRates.find(curMotion.tool);
            double axisRapid[3] = {0, 0, 0};
            for (int k = 0; k < 3; k++) {
                const MachineAxis* axis = mMachine.Linear(k);
                axisRapid[k] = axis ? axis->velocity : 0;
            }
            const double seconds = MotionSeconds(
                prevMotion,
                curMotion,
                cut,
                rates != mToolRates.end() ? rates->second : ToolRates(),
                mRapidRate,
                mMachine.Known() ? axisRapid : nullptr
            );
            mSegmentStartSeconds.push_back(
                mSegmentSeconds.empty() ? 0 : mSegmentStartSeconds.back() + mSegmentSeconds.back()
            );
            mSegmentSeconds.push_back(seconds);
            mSegmentStencilSteps.push_back(segment->numSimSteps);
            mSegmentStencilSweepSteps.push_back(segment->SweepSteps());
            mSegments.push_back(cut);
            mSegmentMotions.push_back(curMotion);
        }

        prevMotion = curMotion;
    }
    ApplyTimeline();
    mMachineEvents = mSegmentMotions.empty()
        ? std::vector<Material::Event>()
        : MachineEvents(mMachine, mCodeParser.Operations.front(), mSegmentMotions);

    // colors of the operation tags: of the first operation with the tag, operations without a
    // color of their own get hues far apart
    mColoring.operationColors.assign(3 * std::min((int)mOperations.size(), Material::MaxTag), 0.0f);
    for (int i = 0; i < (int)mOperations.size() && i < Material::MaxTag; i++) {
        float* rgb = &mColoring.operationColors[3 * i];
        if (mOperations[i].hasColor) {
            std::copy(mOperations[i].color, mOperations[i].color + 3, rgb);
            continue;
        }
        const float hue = std::fmod(i * 0.381966f + 0.05f, 1.0f) * 6;  // golden angle
        const float saturation = 0.55f, value = 0.9f;
        const int sector = (int)hue % 6;
        const float f = hue - std::floor(hue);
        const float p = value * (1 - saturation), q = value * (1 - saturation * f),
                    t = value * (1 - saturation * (1 - f));
        const float table[6][3] = {
            {value, t, p},
            {q, value, p},
            {p, value, t},
            {p, q, value},
            {t, p, value},
            {value, p, q},
        };
        std::copy(table[sector], table[sector] + 3, rgb);
    }

    millPathLine.GenerateModel();

    InitDisplay(quality);
    simDisplay.simCacheValid = false;
    mMaterialNeedsStart = true;

    simulationInitiated = true;
}

void MillSimulation::ApplyTimeline()
{
    const size_t n = mSegments.size();
    mSegmentSteps.resize(n);
    mSegmentFirstStep.resize(n);
    for (SimOperation& op : mOperations) {
        op.firstStep = op.endStep = -1;
    }
    mNTotalSteps = 0;
    for (size_t s = 0; s < n; s++) {
        int steps, sweepSteps;
        if (mStencilRendering) {
            steps = mSegmentStencilSteps[s];
            sweepSteps = mSegmentStencilSweepSteps[s];
        }
        else {
            steps = sweepSteps = std::max(1, (int)std::lround(mSegmentSeconds[s] * StepsPerSecond));
        }
        mSegmentSteps[s] = steps;
        mSegments[s].steps = steps;
        mSegments[s].sweepSteps = sweepSteps;
        mSegmentFirstStep[s] = mNTotalSteps;
        const int op = mSegmentMotions[s].operation;
        if (op >= 0 && op < (int)mOperations.size()) {
            if (mOperations[op].firstStep < 0) {
                mOperations[op].firstStep = mNTotalSteps;
            }
            mOperations[op].endStep = mNTotalSteps + steps;
        }
        mNTotalSteps += steps;
    }
    // operations without motion are empty, where the operation before them ends
    int lastEnd = 0;
    for (SimOperation& op : mOperations) {
        if (op.firstStep < 0) {
            op.firstStep = op.endStep = lastEnd;
        }
        lastEnd = op.endStep;
    }
    mNPathSteps = (int)n;
}

double MillSimulation::CurrentSeconds() const
{
    if (mPathStep < 0 || mPathStep >= (int)mSegmentSeconds.size()) {
        return 0;
    }
    const double part = std::min(1.0, (double)mSubStep / std::max(1, mSegmentSteps[mPathStep]));
    return mSegmentStartSeconds[mPathStep] + part * mSegmentSeconds[mPathStep];
}

double MillSimulation::TotalSeconds() const
{
    return mSegmentSeconds.empty() ? 0 : mSegmentStartSeconds.back() + mSegmentSeconds.back();
}

std::vector<double> MillSimulation::OperationSeconds() const
{
    std::vector<double> seconds(mOperations.size(), 0.0);
    for (size_t s = 0; s < mSegmentSeconds.size(); s++) {
        const int op = mSegmentMotions[s].operation;
        if (op >= 0 && op < (int)seconds.size()) {
            seconds[op] += mSegmentSeconds[s];
        }
    }
    return seconds;
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

void MillSimulation::AddTool(
    const std::vector<float>& toolProfile,
    int toolid,
    float diameter,
    const std::vector<Material::ProfileArc>& arcs,
    double flutes,
    const ToolRates& rates
)
{
    // if we have another tool with same id, remove it
    RemoveTool(toolid);
    EndMill* tool = new EndMill(toolProfile, toolid, diameter);
    mToolTable.push_back(tool);
    mToolArcs[toolid] = arcs;
    mToolFlutes[toolid] = flutes;
    tool->SetFlutes((float)flutes);
    mToolRates[toolid] = rates;
}

void MillSimulation::SetMachine(const MachineKinematics& machine)
{
    mMachine = machine;
    // the rotary words of G-code text are the angles of its axes
    mCodeParser.SetMachine(&mMachine);
}

void MillSimulation::SetRapidRate(double rate)
{
    mRapidRate = rate > 0 ? rate : 5000.0 / 60;
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
    if (mNPathSteps <= 0) {
        mPathStep = -1;
        mSubStep = 0;
        return;
    }
    if (mCurStep >= mNTotalSteps) {
        mPathStep = mNPathSteps - 1;
        mSubStep = mSegmentSteps[mPathStep];
        return;
    }
    // the segment of the step
    mPathStep = (int)(std::upper_bound(mSegmentFirstStep.begin(), mSegmentFirstStep.end(), mCurStep)
                      - mSegmentFirstStep.begin())
        - 1;
    mSubStep = mCurStep - mSegmentFirstStep[mPathStep] + 1;
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
    if (mPathStep < 0 || !mToolVisible) {
        return;
    }

    MillPathSegment* p = MillPathSegments.at(mPathStep);
    if (mStencilRendering || mPathStep >= (int)mSegments.size()) {
        vec3 toolPos;
        p->GetHeadPosition(toolPos);
        mat4x4 tmat;
        mat4x4_translate(tmat, toolPos[0], toolPos[1], toolPos[2]);
        RenderToolShape(p->endmill, tmat, identityMat);
        return;
    }

    // tool at its position and orientation at the current step
    const Material::Segment& segment = mSegments[mPathStep];
    const double t = segment.StepParam(mSubStep);
    double position[3], direction[3];
    segment.PositionAt(t, position);
    segment.AxisAt(t, direction);
    const float pos[3] = {(float)position[0], (float)position[1], (float)position[2]};
    const float axis[3] = {(float)direction[0], (float)direction[1], (float)direction[2]};
    float e1[3] = {1, 0, 0};
    if (std::abs(axis[0]) > 1e-6f || std::abs(axis[1]) > 1e-6f) {
        // perpendicular to the axis, in the plane of the axis and Z
        const float h = std::hypot(axis[0], axis[1]);
        e1[0] = axis[0] * axis[2] / h;
        e1[1] = axis[1] * axis[2] / h;
        e1[2] = -h;
    }
    const float e2[3] = {
        axis[1] * e1[2] - axis[2] * e1[1],
        axis[2] * e1[0] - axis[0] * e1[2],
        axis[0] * e1[1] - axis[1] * e1[0],
    };
    mat4x4 rmat, tmat;
    mat4x4_identity(rmat);
    for (int k = 0; k < 3; k++) {
        rmat[0][k] = e1[k];
        rmat[1][k] = e2[k];
        rmat[2][k] = axis[k];
    }
    mat4x4_translate(tmat, pos[0], pos[1], pos[2]);
    mat4x4_mul(tmat, tmat, rmat);
    RenderToolShape(p->endmill, tmat, rmat);
}

void MillSimulation::RenderToolShape(const EndMill* tool, const mat4x4& tmat, const mat4x4& nmat)
{
    if (!tool->HasFlutes()) {
        // the cutting length is unknown: all of the tool may cut
        simDisplay.StartGeometryPass(fluteColor, false);
        tool->toolShape.Render(tmat, nmat);
        return;
    }
    simDisplay.StartGeometryPass(fluteColor, false);
    tool->fluteShape.Render(tmat, nmat);
    simDisplay.StartGeometryPass(shankColor, false);
    tool->shankShape.Render(tmat, nmat);
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
    // the model a little behind the stock: where the stock is cut to the model, the stock and
    // the colors of its operations are shown, the model only where the stock is cut deeper
    glPolygonOffset(1, 4);
    glEnable(GL_POLYGON_OFFSET_FILL);
    // the section cuts the model too, its cap over the stock's, so it shows in it
    simDisplay.sectionOn = true;
    simDisplay.StartGeometryPass(baseShapeColor, false);
    mBaseShape.render();
    glDisable(GL_POLYGON_OFFSET_FILL);
    if (simDisplay.SectionEnabled()) {
        simDisplay.BeginSectionStencil();
        simDisplay.StartGeometryPass(baseShapeColor, false);
        mBaseShape.render();
        const vec3 cap = {baseShapeColor[0] * 0.8f, baseShapeColor[1] * 0.8f, baseShapeColor[2] * 0.8f};
        float lo[3], hi[3];
        StockBox(lo, hi);
        simDisplay.DrawSectionCap(false, cap, lo, hi);
    }
    simDisplay.sectionOn = false;
}

// The points measured, crosses, and the line between them, seen through the objects
void MillSimulation::RenderMeasure()
{
    if (mInspect != 4 || mMeasure.points == 0) {
        return;
    }
    float lo[3], hi[3];
    StockBox(lo, hi);
    const float size = 0.01f * std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], 1.0f});
    std::vector<float> lines;
    for (int p = 0; p < mMeasure.points; p++) {
        const float* c = p ? mMeasure.b : mMeasure.a;
        for (int k = 0; k < 3; k++) {
            float e[3] = {0, 0, 0};
            e[k] = size;
            lines.insert(lines.end(), {c[0] - e[0], c[1] - e[1], c[2] - e[2]});
            lines.insert(lines.end(), {c[0] + e[0], c[1] + e[1], c[2] + e[2]});
        }
    }
    if (mMeasure.points == 2) {
        lines.insert(lines.end(), mMeasure.a, mMeasure.a + 3);
        lines.insert(lines.end(), mMeasure.b, mMeasure.b + 3);
    }
    if (mMeasureVbo == 0) {
        glGenBuffers(1, &mMeasureVbo);
    }
    glBindBuffer(GL_ARRAY_BUFFER, mMeasureVbo);
    glBufferData(GL_ARRAY_BUFFER, lines.size() * sizeof(float), lines.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    const vec3 color = {1.0f, 0.85f, 0.1f};
    for (bool hidden : {true, false}) {
        simDisplay.SetupLinePass(color, hidden, 0.6f);
        glDrawArrays(GL_LINES, 0, (int)lines.size() / 3);
    }
    glDepthMask(GL_TRUE);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// The axes of the origin, a fifth of the size of the stock long, X red, Y green, Z blue
void MillSimulation::RenderOrigin()
{
    if (!mOriginVisible) {
        return;
    }
    float size = 0;
    for (int k = 0; k < 3; k++) {
        float lo = 1e30f, hi = -1e30f;
        for (size_t i = k; i < mStockXyz.size(); i += 3) {
            lo = std::min(lo, mStockXyz[i]);
            hi = std::max(hi, mStockXyz[i]);
        }
        size = std::max(size, hi - lo);
    }
    const float length = std::max(2.0f, 0.2f * size);
    if (mOriginVbo == 0 || length != mOriginLength) {
        const float lines[] = {0, 0, 0, length, 0, 0, 0, 0, 0, 0, length, 0, 0, 0, 0, 0, 0, length};
        if (mOriginVbo == 0) {
            glGenBuffers(1, &mOriginVbo);
        }
        glBindBuffer(GL_ARRAY_BUFFER, mOriginVbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(lines), lines, GL_STATIC_DRAW);
        mOriginLength = length;
    }
    const vec3 colors[3] = {{0.9f, 0.15f, 0.15f}, {0.15f, 0.75f, 0.2f}, {0.2f, 0.4f, 1.0f}};
    glBindBuffer(GL_ARRAY_BUFFER, mOriginVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    for (bool hidden : {true, false}) {
        for (int axis = 0; axis < 3; axis++) {
            // seen through the stock too, which hides it at a corner of the stock
            simDisplay.SetupLinePass(colors[axis], hidden, 0.6f);
            glDrawArrays(GL_LINES, 2 * axis, 2);
        }
    }
    glDepthMask(GL_TRUE);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void MillSimulation::Render()
{
    if (!simulationInitiated) {
        return;
    }

    // set background
    glClearColor(bgndColor[0], bgndColor[1], bgndColor[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    if (!mStencilRendering) {
        UpdateMaterialSimulation();
    }

    // render the simulation offscreen in an FBO

    if (simDisplay.updateDisplay) {
        // opaque objects first, then the transparent ones over them
        const bool transparentStock = !mStencilRendering && mStockOpacity < 1;
        const bool transparentBase = mBaseOpacity < 1;
        if (mStencilRendering) {
            RenderSimulation();
        }
        else {
            simDisplay.PrepareFrameBuffer();
            if (!transparentStock) {
                RenderMaterialStock();
            }
            if (mMaterial && (mViewItems & VIEWITEM_SIMULATION) != 0) {
                mMaterial->RenderInspection(simDisplay);
            }
        }
        RenderTool();
        if (!transparentBase) {
            RenderBaseShape();
        }
        RenderPath();
        RenderOrigin();
        RenderMeasure();
        simDisplay.ClearTransparentLayer();
        if (transparentStock || transparentBase) {
            for (int layer = 0; simDisplay.StartTransparentLayer(layer); layer++) {
                if (transparentStock && mStockOpacity > 0) {
                    simDisplay.SetOpacity(mStockOpacity);
                    RenderMaterialStock();
                }
                if (transparentBase && mBaseOpacity > 0) {
                    simDisplay.SetOpacity(mBaseOpacity);
                    RenderBaseShape();
                }
                simDisplay.EndTransparentLayer();
            }
        }
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

        // in the time of the machine, the steps of the stencil renderer 60 a second
        const float secondsToSteps = mSimSpeed * (mStencilRendering ? 60.0f : (float)StepsPerSecond);
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

    // the tool waits for the stock: playback doesn't advance while the shown stock is behind
    // the tool, which can happen when cutting and meshing take longer than the playback speed
    // allows
    const bool stockBehind = mMaterial && mMaterial->Started()
        && mMaterial->ShownStep() < mTargetStep;
    if (mSimPlaying && stockBehind) {
        mTotalElapsed = 0s;
        numSteps = 0;
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

    // box mesh for the material simulation, triangles facing outwards
    mStockXyz.clear();
    for (int i = 0; i < 8; i++) {
        mStockXyz.insert(
            mStockXyz.end(),
            {x + (i & 1 ? l : 0), y + (i & 2 ? w : 0), z + (i & 4 ? h : 0)}
        );
    }
    mStockTriangles = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4,
                       2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
    mMaterialNeedsStart = true;
}

void MillSimulation::SetArbitraryStock(
    const std::vector<Vertex>& verts,
    const std::vector<GLushort>& indices
)
{
    mStockObject.GenerateSolid(verts, indices);
    simDisplay.ScaleViewToStock(&mStockObject);
    simDisplay.simCacheValid = false;

    mStockXyz.clear();
    for (const Vertex& v : verts) {
        mStockXyz.insert(mStockXyz.end(), {v.x, v.y, v.z});
    }
    mStockTriangles.assign(indices.begin(), indices.end());
    mMaterialNeedsStart = true;
    ApplySection();
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

void MillSimulation::SetSection(int axis, float position, bool flip)
{
    mSectionAxis = axis >= 0 && axis < 3 ? axis : -1;
    mSectionPosition = std::clamp(position, 0.0f, 1.0f);
    mSectionFlip = flip;
    ApplySection();
}

void MillSimulation::StockBox(float lo[3], float hi[3]) const
{
    for (int k = 0; k < 3; k++) {
        lo[k] = 1e30f;
        hi[k] = -1e30f;
        for (size_t i = k; i < mStockXyz.size(); i += 3) {
            lo[k] = std::min(lo[k], mStockXyz[i]);
            hi[k] = std::max(hi[k], mStockXyz[i]);
        }
    }
}

// the plane of the section in the box of the stock
void MillSimulation::ApplySection()
{
    vec3 normal = {0, 0, 0};
    float offset = 0;
    if (mSectionAxis >= 0 && !mStockXyz.empty()) {
        float lo = 1e30f, hi = -1e30f;
        for (size_t i = mSectionAxis; i < mStockXyz.size(); i += 3) {
            lo = std::min(lo, mStockXyz[i]);
            hi = std::max(hi, mStockXyz[i]);
        }
        const float at = lo + mSectionPosition * (hi - lo);
        const float sign = mSectionFlip ? -1.0f : 1.0f;
        normal[mSectionAxis] = sign;
        offset = sign * at;
    }
    simDisplay.SetSection(mSectionAxis >= 0 && !mStockXyz.empty(), normal, offset);
    simDisplay.updateDisplay = true;
}

void MillSimulation::SetOriginVisible(bool b)
{
    mOriginVisible = b;
    simDisplay.updateDisplay = true;
}

void MillSimulation::SetToolVisible(bool b)
{
    mToolVisible = b;
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

bool MillSimulation::AddGcodeLine(const char* line, int source)
{
    return mCodeParser.AddLine(line, source);
}

bool MillSimulation::AddCommand(
    const std::string& name,
    const std::map<std::string, double>& params,
    int source
)
{
    return mCodeParser.AddCommand(name, params, source);
}

void MillSimulation::BeginOperation(const std::string& name, const float* color)
{
    mCodeParser.currentOperation = (int)mOperations.size();
    SimOperation op {name, 0, 0};
    if (color) {
        op.hasColor = true;
        std::copy(color, color + 3, op.color);
    }
    mOperations.push_back(op);
}

void MillSimulation::SetColorByOperation(bool b)
{
    mColoring.byOperation = b;
    simDisplay.updateDisplay = true;
}

void MillSimulation::SetOpacity(float stock, float base)
{
    mStockOpacity = std::clamp(stock, 0.0f, 1.0f);
    mBaseOpacity = std::clamp(base, 0.0f, 1.0f);
    simDisplay.updateDisplay = true;
}

void MillSimulation::SetThinWalls(bool enabled, float threshold)
{
    mThinWalls = enabled;
    mThinThreshold = threshold;
    // the thickness is only measured while it is shown, it costs time
    if (mMaterial) {
        mMaterial->SetMeasureThickness(enabled);
    }
    mColoring.thinThreshold = enabled ? threshold : 0;
    simDisplay.updateDisplay = true;
}

void MillSimulation::SetOutlines(bool enabled)
{
    simDisplay.outlines = enabled;
    simDisplay.updateDisplay = true;
}

void MillSimulation::SetDeviation(bool enabled, float tolerance)
{
    mDeviation = enabled;
    mColoring.deviation = enabled;
    mColoring.tolerance = tolerance;
    // the distance is only measured while it is shown, it costs time
    if (mMaterial) {
        mMaterial->SetDeviation(mDeviation ? mDeviationModel : nullptr);
        mMaterial->SetModel(mDeviationModel, tolerance);
    }
    simDisplay.updateDisplay = true;
}

void MillSimulation::SetDeviationModel(std::shared_ptr<const Material::ModelDistance> model)
{
    mDeviationModel = std::move(model);
    if (mMaterial) {
        mMaterial->SetDeviation(mDeviation ? mDeviationModel : nullptr);
        mMaterial->SetModel(mDeviationModel, mColoring.tolerance);
    }
    simDisplay.updateDisplay = true;
}

void MillSimulation::SetInspect(int mode)
{
    mInspect = std::clamp(mode, 0, 4);
    mMeasure = Measure();
    ApplyInspect();
    simDisplay.updateDisplay = true;
}

void MillSimulation::ApplyInspect()
{
    if (!mMaterial) {
        return;
    }
    // normals of a length visible on the whole stock
    float size = 0;
    for (int k = 0; k < 3; k++) {
        float lo = 1e30f, hi = -1e30f;
        for (size_t i = k; i < mStockXyz.size(); i += 3) {
            lo = std::min(lo, mStockXyz[i]);
            hi = std::max(hi, mStockXyz[i]);
        }
        size = std::max(size, hi - lo);
    }
    const auto inspect = mInspect <= 3 ? (Material::View::Inspect)mInspect : Material::View::Inspect::None;
    mMaterial->SetInspect(inspect, std::max(size * 0.006f, 0.01f));
}

int MillSimulation::OperationAt(int step) const
{
    // the last operation started at the step, empty operations have no steps
    int found = -1;
    for (int i = 0; i < (int)mOperations.size(); i++) {
        if (mOperations[i].endStep > mOperations[i].firstStep && mOperations[i].firstStep <= step) {
            found = i;
        }
    }
    return found;
}

void MillSimulation::GoToOperation(int direction)
{
    if (direction > 0) {
        for (const SimOperation& op : mOperations) {
            if (op.endStep > op.firstStep && op.firstStep > mCurStep) {
                SetSimulationStep(op.firstStep);
                return;
            }
        }
        SetSimulationStep(mNTotalSteps);
        return;
    }
    int target = 0;
    for (const SimOperation& op : mOperations) {
        if (op.endStep > op.firstStep && op.firstStep < mCurStep) {
            target = op.firstStep;
        }
    }
    SetSimulationStep(target);
}

void MillSimulation::GoToOperationIndex(int index)
{
    if (index < 0 || index >= (int)mOperations.size()) {
        return;
    }
    SetSimulationStep(mOperations[index].firstStep);
}

int MillSimulation::CurrentCommand() const
{
    if (mPathStep < 0 || mPathStep >= (int)mSegmentMotions.size()) {
        return -1;
    }
    return mSegmentMotions[mPathStep].source;
}

void MillSimulation::GoToCommand(int source)
{
    // the commands of the segments are in the order of the toolpath
    for (size_t s = 0; s < mSegmentMotions.size(); s++) {
        if (mSegmentMotions[s].source >= source) {
            SetSimulationStep(mSegmentFirstStep[s]);
            return;
        }
    }
    SetSimulationStep(mNTotalSteps);
}

void MillSimulation::StepCommand(int direction)
{
    const int current = CurrentCommand();
    if (direction > 0) {
        GoToCommand(current + 1);
        return;
    }
    // the first segment of the command in progress
    int first = std::clamp(mPathStep, 0, (int)mSegmentMotions.size());
    while (first > 0 && mSegmentMotions[first - 1].source == current) {
        first--;
    }
    if (first >= (int)mSegmentFirstStep.size()) {
        return;
    }
    if (mCurStep > mSegmentFirstStep[first] || first == 0) {
        SetSimulationStep(mSegmentFirstStep[first]);
        return;
    }
    GoToCommand(mSegmentMotions[first - 1].source);
}

void MillSimulation::StockTriangles(std::vector<float>& xyz, std::vector<uint32_t>& triangles) const
{
    if (mMaterial) {
        mMaterial->Triangles(xyz, triangles);
    }
}

std::vector<SimEvent> MillSimulation::GetEvents() const
{
    std::vector<SimEvent> events;
    std::vector<Material::Event> found = mMachineEvents;
    if (mMaterial) {
        const auto material = mMaterial->Events();
        found.insert(found.end(), material.begin(), material.end());
    }
    for (const Material::Event& e : found) {
        SimEvent se;
        se.event = e;
        if (e.firstSegment >= 0 && e.firstSegment < (int)mSegmentFirstStep.size()) {
            se.step = mSegmentFirstStep[e.firstSegment];
            se.command = mSegmentMotions[e.firstSegment].source;
        }
        else if (e.operation >= 0 && e.operation < (int)mOperations.size()) {
            // regions of the final stock: where the operation that cut them is done
            se.step = mOperations[e.operation].endStep;
        }
        events.push_back(se);
    }
    // in the order of the timeline
    std::stable_sort(events.begin(), events.end(), [](const SimEvent& a, const SimEvent& b) {
        return a.step < b.step;
    });
    return events;
}

int MillSimulation::EventCount() const
{
    return (int)mMachineEvents.size() + (mMaterial ? mMaterial->EventCount() : 0);
}

void MillSimulation::GoToEvent(int index)
{
    const std::vector<SimEvent> events = GetEvents();
    if (index >= 0 && index < (int)events.size()) {
        SetSimulationStep(events[index].step);
    }
}

void MillSimulation::StepEvent(int direction)
{
    const std::vector<SimEvent> events = GetEvents();
    int target = -1;
    for (const SimEvent& e : events) {
        if (direction > 0 && e.step > mCurStep && (target < 0 || e.step < target)) {
            target = e.step;
        }
        if (direction < 0 && e.step < mCurStep && e.step > target) {
            target = e.step;
        }
    }
    if (target >= 0) {
        SetSimulationStep(target);
    }
}

void MillSimulation::RequestPick(int x, int y)
{
    simDisplay.RequestPick(x, y);
}

bool MillSimulation::TakePick(int& operation, bool& stock, float position[3])
{
    PickResult pick;
    if (!simDisplay.TakePick(pick)) {
        return false;
    }
    stock = pick.stock;
    if (pick.stock && mInspect == 3 && mMaterial) {
        mMaterial->InspectAt(pick.position);
        simDisplay.updateDisplay = true;
    }
    if (pick.hit && mInspect == 4) {
        // the first point again after two
        if (mMeasure.points != 1) {
            mMeasure = Measure();
            std::copy(pick.position, pick.position + 3, mMeasure.a);
            mMeasure.points = 1;
            // the wall of the stock behind it, against its normal
            if (pick.stock && mMaterial) {
                const float inward[3] = {-pick.normal[0], -pick.normal[1], -pick.normal[2]};
                mMeasure.wall = mMaterial->RayDistance(pick.position, inward);
            }
        }
        else {
            std::copy(pick.position, pick.position + 3, mMeasure.b);
            mMeasure.points = 2;
        }
        simDisplay.updateDisplay = true;
    }
    // operations share the tags beyond Material::MaxTag
    operation = pick.stock && pick.tag > 0 && pick.tag <= (int)mOperations.size() ? pick.tag - 1 : -1;
    std::copy(pick.position, pick.position + 3, position);
    return true;
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
    SetSimulationStep((int)((float)mNTotalSteps * stage));
}

void MillSimulation::SetSimulationStep(int step)
{
    const int newStep = std::clamp(step, 0, mNTotalSteps);
    if (newStep == mCurStep) {
        return;
    }

    mCurStep = newStep;
    CalcSegmentPositions();

    simDisplay.updateDisplay = true;
}

// True while the displayed simulation does not yet match the current simulation position, for
// renderers that spread the work over several frames.
bool MillSimulation::IsRenderPending() const
{
    return !mStencilRendering && (mMaterialNeedsStart || (mMaterial && mMaterial->Pending()));
}

bool MillSimulation::NeedsFrame() const
{
    if (mStencilRendering) {
        return mSimPlaying || mSingleStep || simDisplay.updateDisplay;
    }
    // the simulation and the mesh are made in other threads, a frame shows a new mesh
    return mSimPlaying || mSingleStep || simDisplay.updateDisplay || mMaterialNeedsStart
        || (mMaterial && mMaterial->HasNewMesh());
}

void MillSimulation::SetStencilRendering(bool b)
{
    if (b == mStencilRendering) {
        return;
    }
    mStencilRendering = b;
    mMaterialNeedsStart = !b;
    if (b) {
        mMaterial.reset();
    }
    // the steps of the other renderer, at the same place
    const double stage = mNTotalSteps > 0 ? (double)mCurStep / mNTotalSteps : 0;
    ApplyTimeline();
    mCurStep = std::clamp((int)std::lround(stage * mNTotalSteps), 0, mNTotalSteps);
    CalcSegmentPositions();
    simDisplay.simCacheValid = false;
    simDisplay.updateDisplay = true;
}

// The input of the material simulation: tools by their id, segments refer to them by index
Material::Job MillSimulation::MaterialJob() const
{
    Material::Job job;
    job.quality = mQuality;
    std::vector<int> toolIds;
    for (EndMill* tool : mToolTable) {
        job.profiles.push_back(tool->rawProfile);
        const auto arcs = mToolArcs.find(tool->toolId);
        job.arcs.push_back(arcs != mToolArcs.end() ? arcs->second : std::vector<Material::ProfileArc>());
        const auto flutes = mToolFlutes.find(tool->toolId);
        job.flutes.push_back(flutes != mToolFlutes.end() ? flutes->second : 0);
        toolIds.push_back(tool->toolId);
    }
    job.segments = mSegments;
    for (size_t i = 0; i < job.segments.size(); i++) {
        const int id = MillPathSegments[i]->endmill->toolId;
        job.segments[i].tool = (int)(std::find(toolIds.begin(), toolIds.end(), id) - toolIds.begin());
    }
    job.stockXyz = mStockXyz;
    job.stockTriangles = mStockTriangles;
    return job;
}

void MillSimulation::StartMaterialSimulation()
{
    mMaterialNeedsStart = false;
    if (mStockXyz.empty() || mSegments.size() != MillPathSegments.size()) {
        mMaterial.reset();
        return;
    }
    Material::Job job = MaterialJob();
    if (const char* dumpPath = std::getenv("CAM_SIM_DUMP")) {
        Material::SaveJob(dumpPath, job);
    }
    std::vector<std::shared_ptr<const Material::Tool>> tools;
    for (size_t t = 0; t < job.profiles.size(); t++) {
        tools.push_back(std::make_shared<Material::Tool>(job.profiles[t], true, job.arcs[t]));
    }
    if (!mMaterial) {
        mMaterial = std::make_unique<Material::View>();
    }
    mMaterial->SetFluteHeights(job.flutes);
    mMaterial->SetModel(mDeviationModel, mColoring.tolerance);
    mMaterial->SetMeasureThickness(mThinWalls);
    mMaterial->SetDeviation(mDeviation ? mDeviationModel : nullptr);
    ApplyInspect();
    mMaterial->Start(std::move(tools), std::move(job.segments), mStockXyz, mStockTriangles, mQuality);
    mTargetStep = 0;
}

void MillSimulation::UpdateMaterialSimulation()
{
    if (mMaterialNeedsStart) {
        StartMaterialSimulation();
        simDisplay.updateDisplay = true;
    }
    if (!mMaterial || !mMaterial->Started()) {
        return;
    }
    const long target = mPathStep < 0 ? 0 : mMaterial->GlobalStep(mPathStep, mSubStep);
    mMaterial->SetTarget(target);
    mTargetStep = target;
    if (mMaterial->Update()) {
        simDisplay.updateDisplay = true;
    }
}

void MillSimulation::RenderMaterialStock()
{
    if ((mViewItems & VIEWITEM_SIMULATION) == 0) {
        return;
    }
    if (mMaterial) {
        // the section cuts the stock, its cap in a darker stock color
        simDisplay.sectionOn = true;
        mMaterial->Render(simDisplay, stockColor, cutColor, mColoring);
        if (simDisplay.SectionEnabled()) {
            simDisplay.BeginSectionStencil();
            mMaterial->Render(simDisplay, stockColor, cutColor, mColoring);
            const vec3 cap = {stockColor[0] * 0.75f, stockColor[1] * 0.75f, stockColor[2] * 0.75f};
            float lo[3], hi[3];
            StockBox(lo, hi);
            simDisplay.DrawSectionCap(true, cap, lo, hi);
        }
        simDisplay.sectionOn = false;
    }
}

void MillSimulation::SetScrubbing(bool b)
{
    if (b == mScrubbing) {
        return;
    }

    mScrubbing = b;
    // the stencil renderer replaces the incremental result after scrubbing, the material
    // simulation shows the same result while scrubbing and at rest
    if (mStencilRendering) {
        simDisplay.updateDisplay = true;
    }
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
    simDisplay.updateDisplay = true;
}


}  // namespace CAMSimulator
