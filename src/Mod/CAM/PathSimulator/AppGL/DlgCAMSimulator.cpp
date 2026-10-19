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


#include "DlgCAMSimulator.h"

#include "Dummy3DViewer.h"
#include "GuiDisplay.h"
#include "MaterialModel.h"
#include "MillSimulation.h"
#include "ViewCAMSimulator.h"
#include <App/Application.h>
#include <Gui/View3DInventorViewer.h>
#include <Inventor/nodes/SoCamera.h>
#include <BRepBuilderAPI_Copy.hxx>
#include <QSurfaceFormat>
#include <deque>
#include <limits>
#include <numeric>
#include <utility>

// include this last as the defines can mess up other includes
#include "OpenGlWrapper.h"

using namespace std::literals;

namespace CAMSimulator
{

ParameterGrp::handle CAMPreferences()
{
    return App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/CAM");
}

float SimShape::maxDimension() const
{
    float xmin = NAN, ymin = NAN, zmin = NAN;
    float xmax = NAN, ymax = NAN, zmax = NAN;

    for (const auto& v : verts) {
        xmin = std::fmin(xmin, v.x);
        ymin = std::fmin(ymin, v.x);
        zmin = std::fmin(zmin, v.x);

        xmax = std::fmax(xmax, v.x);
        ymax = std::fmax(ymax, v.x);
        zmax = std::fmax(zmax, v.x);
    }

    const float xsize = xmax - xmin;
    const float ysize = ymax - ymin;
    const float zsize = zmax - zmin;

    return std::max(std::max(xsize, ysize), zsize);
}


DlgCAMSimulator::DlgCAMSimulator(QWidget* parent)
    : QOpenGLWidget(parent)
{
    QSurfaceFormat format = QSurfaceFormat::defaultFormat();
    int samples = Gui::View3DInventorViewer::getNumSamples();
    if (samples > 1) {
        format.setSamples(samples);
    }
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    setFormat(format);

    setMouseTracking(true);

    mMillSimulator.reset(new MillSimulation);

    // the handler runs after painting, it may change other widgets
    connect(
        this,
        &DlgCAMSimulator::operationPicked,
        this,
        [this](int operation) {
            if (mOperationPicked) {
                mOperationPicked(operation);
            }
        },
        Qt::QueuedConnection
    );
}

DlgCAMSimulator::~DlgCAMSimulator()
{
    makeCurrent();
    mMillSimulator = nullptr;
}

void DlgCAMSimulator::goToOperation(int index)
{
    mMillSimulator->GoToOperationIndex(index);
    updateGui();
    update();
}

void DlgCAMSimulator::pickAt(const QPoint& pos)
{
    const qreal ratio = devicePixelRatioF();
    const int x = (int)(pos.x() * ratio);
    const int y = (int)((height() - 1 - pos.y()) * ratio);
    mMillSimulator->RequestPick(x, y);
    update();
}

void DlgCAMSimulator::setOperationPickedHandler(std::function<void(int)> handler)
{
    mOperationPicked = std::move(handler);
}

void DlgCAMSimulator::connectTo(GuiDisplay& gui, Dummy3DViewer& dv)
{
    // connect to gui

    mGui = &gui;
    updateGui();

    connect(&gui, &GuiDisplay::play, this, [this](bool b) { mMillSimulator->SetPlaying(b); });

    connect(&gui, &GuiDisplay::singleStep, this, [this] { mMillSimulator->SingleStep(); });

    connect(&gui, &GuiDisplay::skipToStart, this, [this] {
        mMillSimulator->SetSimulationStep(0);
        updateGui();
    });

    connect(&gui, &GuiDisplay::skipToEnd, this, [this] {
        mMillSimulator->SetSimulationStep(mMillSimulator->GetState().mNTotalSteps);
        updateGui();
    });

    connect(&gui, &GuiDisplay::previousOperation, this, [this] {
        mMillSimulator->GoToOperation(-1);
        updateGui();
    });

    connect(&gui, &GuiDisplay::nextOperation, this, [this] {
        mMillSimulator->GoToOperation(1);
        updateGui();
    });

    connect(&gui, &GuiDisplay::speedChanged, this, [this](int speed) {
        mMillSimulator->SetSpeed(speed);
    });

    connect(&gui, &GuiDisplay::stageChanged, this, [this](float f) {
        mMillSimulator->SetSimulationStage(f);
    });

    connect(&gui, &GuiDisplay::scrubbing, this, [this](bool b) { mMillSimulator->SetScrubbing(b); });

    connect(&gui, &GuiDisplay::rotateEnableChanged, this, &DlgCAMSimulator::setRotateEnabled);

    connect(&gui, &GuiDisplay::pathVisibleChanged, this, [this](bool b) {
        mMillSimulator->SetPathVisible(b);
    });

    connect(&gui, &GuiDisplay::ssaoEnableChanged, this, [this](bool b) {
        mMillSimulator->EnableSsao(b);
    });


    // connect to dummy viewer

    mDummyViewer = &dv;

    mDummyViewer->setStockVisible(mMillSimulator->IsStockVisible());
    mDummyViewer->setBaseVisible(mMillSimulator->IsBaseVisible());

    // connect gui and dummy viewer

    connect(
        &gui,
        &GuiDisplay::viewAll,
        &dv,
        static_cast<void (Dummy3DViewer::*)()>(&Dummy3DViewer::viewAll)
    );
}

void DlgCAMSimulator::updateGui()
{
    if (!mGui) {
        return;
    }

    const auto state = mMillSimulator->GetState();

    mGui->setPlaying(state.mSimPlaying);
    mGui->setSpeed(state.mSimSpeed);

    const float stage = (float)state.mCurStep / state.mNTotalSteps;
    mGui->setStage(stage, state.mNTotalSteps);

    const auto& operations = mMillSimulator->GetOperations();
    const int operation = mMillSimulator->OperationAt(state.mCurStep);
    mGui->setOperation(
        operation >= 0 ? QString::fromStdString(operations[operation].name) : QString(),
        operation,
        (int)operations.size()
    );

    mGui->setPathVisible(state.mViewPath);
    mGui->setSsaoEnabled(state.mViewSSAO);

    if (mDummyViewer && !mDummyViewer->isAnimating()) {
        mGui->setRotateEnabled(false);
    }
}

void DlgCAMSimulator::cloneFrom(const DlgCAMSimulator& from)
{
    mNeedsInitialize = true;
    mNeedsClear = true;
    setAnimating(from.mAnimating);

    mQuality = from.mQuality;

    mCommands = from.mCommands;
    mSourceCount = from.mSourceCount;
    mOperations = from.mOperations;
    mTools = from.mTools;

    mStock = from.mStock;
    mStock.needsUpdate = true;

    mBase = from.mBase;
    mBase.needsUpdate = true;

    const auto state = from.mMillSimulator->GetState();
    mState = std::make_unique<MillSimulationState>(state);
    mOperationPicked = from.mOperationPicked;
}

DlgCAMSimulator* DlgCAMSimulator::instance(Gui::Document* doc)
{
    return &ViewCAMSimulator::instance(doc).dlg();
}

void DlgCAMSimulator::setAnimating(bool animating)
{
    if (animating == mAnimating) {
        return;
    }

    mAnimating = animating;

    if (animating && mAnimatingTimer == 0) {
        mAnimatingTimer = startTimer(5);
    }
    else if (!animating && mAnimatingTimer != 0) {
        killTimer(mAnimatingTimer);
        mAnimatingTimer = 0;
    }
}

void DlgCAMSimulator::startSimulation(const Part::TopoShape& stock, float quality)
{
    mQuality = quality;
    mNeedsInitialize = true;

    setStockShape(stock, 1);
    setAnimating(true);

    Q_EMIT simulationStarted();
}

void DlgCAMSimulator::resetSimulation()
{
    mNeedsClear = true;

    mCommands.clear();
    mSourceCount = 0;
    mOperations.clear();
    mTools.clear();
    mStock = {};
    mBase = {};
}

void DlgCAMSimulator::addCommand(const std::string& name, const std::map<std::string, double>& params)
{
    mCommands.push_back({name, params, mSourceCount++});
}

void DlgCAMSimulator::addOperation(const char* name, const float* color)
{
    Operation op {mCommands.size(), name, color != nullptr, {0, 0, 0}};
    if (color) {
        std::copy(color, color + 3, op.color);
    }
    mOperations.push_back(op);
}

void DlgCAMSimulator::addTool(
    const std::vector<float>& toolProfilePoints,
    int toolNumber,
    float diameter,
    float resolution
)
{
    Q_UNUSED(resolution)

    mCommands.push_back({"T" + std::to_string(toolNumber), {}, -1});
    mTools.emplace_back(toolProfilePoints, toolNumber, diameter);
}

static SimShape getMeshData(const Part::TopoShape& shape, float resolution)
{
    SimShape ret;

    std::vector<int> normalCount;
    int nVerts = 0;
    for (auto& shape : shape.getSubTopoShapes(TopAbs_FACE)) {
        std::vector<Base::Vector3d> points;
        std::vector<Data::ComplexGeoData::Facet> facets;
        shape.getFaces(points, facets, resolution);

        std::vector<Base::Vector3d> normals(points.size());
        std::vector<int> normalCount(points.size());

        // copy triangle indices and calculate normals
        for (auto face : facets) {
            ret.indices.push_back(face.I1 + nVerts);
            ret.indices.push_back(face.I2 + nVerts);
            ret.indices.push_back(face.I3 + nVerts);

            // calculate normal
            Base::Vector3d vAB = points[face.I2] - points[face.I1];
            Base::Vector3d vAC = points[face.I3] - points[face.I1];
            Base::Vector3d vNorm = vAB.Cross(vAC).Normalize();

            normals[face.I1] += vNorm;
            normals[face.I2] += vNorm;
            normals[face.I3] += vNorm;

            normalCount[face.I1]++;
            normalCount[face.I2]++;
            normalCount[face.I3]++;
        }

        // copy points and set normals
        for (unsigned int i = 0; i < points.size(); i++) {
            Base::Vector3d& point = points[i];
            Base::Vector3d& normal = normals[i];
            int count = normalCount[i];
            normal /= count;
            ret.verts.push_back(Vertex(point.x, point.y, point.z, normal.x, normal.y, normal.z));
        }

        nVerts = ret.verts.size();
    }

    ret.needsUpdate = true;
    return ret;
}

void DlgCAMSimulator::setStockShape(const Part::TopoShape& shape, float resolution)
{
    mStock = getMeshData(shape, resolution);

    if (mDummyViewer) {
        mDummyViewer->setStockShape(shape);
    }

    update();
}

void DlgCAMSimulator::setStockVisible(bool b)
{
    if (b == mMillSimulator->IsStockVisible()) {
        return;
    }

    mMillSimulator->SetStockVisible(b);

    if (mDummyViewer) {
        mDummyViewer->setStockVisible(b);
    }

    update();
}

void DlgCAMSimulator::setBaseShape(const Part::TopoShape& shape, float resolution)
{
    mBase = getMeshData(shape, resolution);
    mBaseShape = shape;
    mDeviationGeneration++;
    mMillSimulator->SetDeviationModel(nullptr);
    if (mDeviation) {
        startDeviationModel();
    }

    if (mDummyViewer) {
        mDummyViewer->setBaseShape(shape);
    }

    update();
}

void DlgCAMSimulator::setBaseVisible(bool b)
{
    if (b == mMillSimulator->IsBaseVisible()) {
        return;
    }

    mMillSimulator->SetBaseVisible(b);

    if (mDummyViewer) {
        mDummyViewer->setBaseVisible(b);
    }

    update();
}

// this is very similar to DemoMode::getDirection in Gui/DemoMode.cpp

static SbVec3f getRotationDirection(Gui::View3DInventorViewer* viewer)
{
    const SbVec3f viewAxis = {0, 0, -1};

    SoCamera* cam = viewer->getSoRenderManager()->getCamera();
    if (!cam) {
        return viewAxis;
    }
    SbRotation rot = cam->orientation.getValue();
    SbRotation inv = rot.inverse();
    SbVec3f vec(viewAxis);
    inv.multVec(vec, vec);
    if (vec.length() < std::numeric_limits<float>::epsilon()) {
        vec = viewAxis;
    }
    vec.normalize();
    return vec;
}

void DlgCAMSimulator::setRotateEnabled(bool b)
{
    if (b) {
        mDummyViewer->startSpinningAnimation(getRotationDirection(mDummyViewer), 0.5f);
    }
    else {
        mDummyViewer->stopAnimating();
    }
}

void DlgCAMSimulator::setBackgroundColor(const QColor& c)
{
    mMillSimulator->SetBackgroundColor({c.redF(), c.greenF(), c.blueF()});
    update();
}

void DlgCAMSimulator::setPathColor(const QColor& normal, const QColor& rapid)
{
    const vec3 vnormal = {normal.redF(), normal.greenF(), normal.blueF()};
    const vec3 vrapid = {rapid.redF(), rapid.greenF(), rapid.blueF()};
    mMillSimulator->SetPathColor(vnormal, vrapid);
    update();
}

void DlgCAMSimulator::setStencilRendering(bool b)
{
    mMillSimulator->SetStencilRendering(b);
    update();
}

void DlgCAMSimulator::setColorByOperation(bool b)
{
    mMillSimulator->SetColorByOperation(b);
    updateGui();
    update();
}

void DlgCAMSimulator::setOpacity(float stock, float model)
{
    mMillSimulator->SetOpacity(stock, model);
    update();
}

void DlgCAMSimulator::setDeviation(bool enabled, float tolerance)
{
    mDeviation = enabled;
    mMillSimulator->SetDeviation(enabled, tolerance);
    if (mGui) {
        mGui->setDeviationScale(enabled, tolerance);
    }
    if (enabled && mDeviationModelGeneration != mDeviationGeneration) {
        startDeviationModel();
    }
    update();
}

void DlgCAMSimulator::setOutlines(bool enabled)
{
    mMillSimulator->SetOutlines(enabled);
    update();
}

void DlgCAMSimulator::setInspect(int mode)
{
    mMillSimulator->SetInspect(mode);
    update();
}

// The model tessellated within 2 um, on a copy so its own tessellation stays, and its triangles
// sorted for the distance queries, in another thread
void DlgCAMSimulator::startDeviationModel()
{
    if (mBaseShape.isNull() || mDeviationModel.valid()) {
        return;
    }
    const int generation = mDeviationGeneration;
    mDeviationModelGeneration = generation;
    const TopoDS_Shape copy = BRepBuilderAPI_Copy(mBaseShape.getShape()).Shape();
    mDeviationModel = std::async(std::launch::async, [copy]() {
        std::vector<Base::Vector3d> points;
        std::vector<Data::ComplexGeoData::Facet> facets;
        Part::TopoShape(copy).getFaces(points, facets, 0.002);
        std::vector<float> xyz;
        std::vector<uint32_t> triangles;
        xyz.reserve(3 * points.size());
        for (const Base::Vector3d& p : points) {
            xyz.insert(xyz.end(), {(float)p.x, (float)p.y, (float)p.z});
        }
        triangles.reserve(3 * facets.size());
        for (const auto& f : facets) {
            triangles.insert(triangles.end(), {f.I1, f.I2, f.I3});
        }
        return std::shared_ptr<const Material::ModelDistance>(
            std::make_shared<Material::ModelDistance>(xyz, triangles, 1.0f)
        );
    });
    // look for the result until it is there
    auto poll = std::make_shared<std::function<void()>>();
    *poll = [this, poll, generation]() {
        if (!mDeviationModel.valid()) {
            return;
        }
        if (mDeviationModel.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            QTimer::singleShot(100, this, *poll);
            return;
        }
        auto model = mDeviationModel.get();
        if (generation == mDeviationGeneration) {
            mMillSimulator->SetDeviationModel(model);
            update();
        }
        else if (mDeviation) {
            startDeviationModel();
        }
    };
    QTimer::singleShot(100, this, *poll);
}

void DlgCAMSimulator::setThinWalls(bool enabled, float threshold)
{
    mMillSimulator->SetThinWalls(enabled, threshold);
    updateGui();
    update();
}

void DlgCAMSimulator::timerEvent(QTimerEvent* event)
{
    (void)event;

    // only render when something changed, so an idle simulator does not use the CPU and GPU
    bool needsFrame = mNeedsInitialize || mNeedsClear || mStock.needsUpdate || mBase.needsUpdate
        || mState || mLastCommand != mCommands.size() || mMillSimulator->NeedsFrame();
    if (mDummyViewer) {
        const SoCamera* camera = mDummyViewer->getCamera();
        if (camera && (camera != mLastCamera || camera->getNodeId() != mLastCameraId)) {
            mLastCamera = camera;
            mLastCameraId = camera->getNodeId();
            needsFrame = true;
        }
    }
    if (needsFrame) {
        update();
    }

    updateGui();
}

void DlgCAMSimulator::updateResources()
{
    // clear simulator

    if (mNeedsClear) {
        mMillSimulator->Clear();
        mLastCommand = 0;
        mLastOperation = 0;
        mNeedsClear = false;
    }

    // update the toolpath

    auto beginOperations = [this](std::size_t line) {
        while (mLastOperation < mOperations.size()
               && mOperations[mLastOperation].firstCommand <= line) {
            const Operation& op = mOperations[mLastOperation];
            mMillSimulator->BeginOperation(op.name, op.hasColor ? op.color : nullptr);
            mLastOperation++;
        }
    };
    for (std::size_t i = mLastCommand; i < mCommands.size(); i++) {
        beginOperations(i);
        const Command& cmd = mCommands[i];
        mMillSimulator->AddCommand(cmd.name, cmd.params, cmd.source);
    }
    beginOperations(mCommands.size());

    mLastCommand = mCommands.size();

    // update tools

    for (const auto& tool : mTools) {
        if (!mMillSimulator->ToolExists(tool.id)) {
            mMillSimulator->AddTool(tool.profile, tool.id, tool.diameter);
        }
    }

    // initialize simulator

    if (mNeedsInitialize) {
        // TODO: mStock is set when we arrive here, still this could be handled nicer
        const float maxStockDimension = mStock.maxDimension();

        mMillSimulator->InitSimulation(mQuality, maxStockDimension);
        mNeedsInitialize = false;

        mLastProcessSim = clock::time_point::min();
    }

    // update stock and base

    if (mStock.needsUpdate) {
        mMillSimulator->SetArbitraryStock(mStock.verts, mStock.indices);
        mStock.needsUpdate = false;
    }

    if (mBase.needsUpdate) {
        mMillSimulator->SetBaseObject(mBase.verts, mBase.indices);
        mBase.needsUpdate = false;
    }

    // update state

    if (mState) {
        mMillSimulator->SetState(*mState);
        mState = nullptr;
    }
}

void DlgCAMSimulator::updateWindowScale()
{
    const qreal ratio = devicePixelRatioF();
    mMillSimulator->UpdateWindowScale(width() * ratio, height() * ratio);
}

void DlgCAMSimulator::updateCamera()
{
    if (!mDummyViewer) {
        return;
    }

    const SoCamera& camera = *mDummyViewer->getCamera();
    mMillSimulator->UpdateCamera(camera);
}

void DlgCAMSimulator::initializeGL()
{
    gOpenGLFunctions.initializeOpenGLFunctions();

    auto glString = [](GLenum name) {
        const auto str = reinterpret_cast<const char*>(glGetString(name));
        return std::string(str ? str : "");
    };
    mGlVendor = glString(GL_VENDOR);
    mGlRenderer = glString(GL_RENDERER);
    mGlVersion = glString(GL_VERSION);
}

void DlgCAMSimulator::setFrameTiming(bool b)
{
    mFrameTiming = b;
    mFrameTimes.clear();
}

std::vector<double> DlgCAMSimulator::takeFrameTimes()
{
    return std::exchange(mFrameTimes, {});
}

bool DlgCAMSimulator::isRenderPending() const
{
    return mMillSimulator->IsRenderPending();
}

std::map<std::string, std::string> DlgCAMSimulator::rendererInfo() const
{
    const qreal ratio = devicePixelRatioF();
    return {
        {"vendor", mGlVendor},
        {"renderer", mGlRenderer},
        {"version", mGlVersion},
        {"width", std::to_string((int)(width() * ratio))},
        {"height", std::to_string((int)(height() * ratio))},
    };
}

void DlgCAMSimulator::paintGL()
{
    const auto frameStart = clock::now();

    updateResources();

    // We need to call updateWindowScale on every render since the devicePixelRatio we get in
    // resizeGL might be wrong on the first resize.

    updateWindowScale();
    updateCamera();

    const auto now = clock::now();
    // frames are only rendered when something changes, limit the time a frame advances the
    // simulation after a pause
    const auto elapsed = mLastProcessSim != clock::time_point::min()
        ? std::min<clock::duration>(now - mLastProcessSim, 250ms)
        : 0s;

#if 0

    static std::deque<clock::duration> q;
    while (q.size() >= 60) {
        q.pop_front();
    }

    q.push_back(elapsed);

    const auto average = std::accumulate(q.begin(), q.end(), clock::duration()) / (float)q.size();

    std::cerr
        << "elapsed: "
        << std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(average).count()
        << "ms" << std::endl;

#endif


    mMillSimulator->ProcessSim(elapsed);

    mLastProcessSim = now;

    // the operation picked, handled after painting
    int operation = -1;
    bool stock = false;
    float position[3];
    // inspecting the dexels, a click chooses where, it doesn't go to the operation
    if (mMillSimulator->TakePick(operation, stock, position) && !mMillSimulator->InspectsDexels()) {
        Q_EMIT operationPicked(operation);
    }

    if (mFrameTiming) {
        glFinish();
        const std::chrono::duration<double, std::milli> frameTime = clock::now() - frameStart;
        mFrameTimes.push_back(frameTime.count());
    }
}

void DlgCAMSimulator::resizeGL(int w, int h)
{
    (void)w, (void)h;
}

}  // namespace CAMSimulator
