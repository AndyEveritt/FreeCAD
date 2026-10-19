// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2017 Shai Seger <shaise at gmail>                       *
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

#ifdef _MSC_VER
# pragma warning(disable : 4251)
#endif

#include <queue>
#include <functional>
#include <future>
#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <QOpenGLWidget>
#include <QPainter>
#include <QTimer>
#include <QExposeEvent>
#include <QResizeEvent>
#include <QMouseEvent>
#include <QOpenGLContext>

#include <Base/Parameter.h>
#include "MaterialTool.h"
#include <Mod/Part/App/TopoShape.h>

class SoCamera;

namespace Gui
{
class MDIView;
class Document;
}  // namespace Gui

namespace CAMSimulator
{

// use short declaration as using 'include' causes a header loop
class MillSimulation;
struct MillSimulationState;
struct SimEvent;
struct Vertex;
namespace Material
{
class ModelDistance;
}
class ViewCAMSimulator;
class GuiDisplay;
class Dummy3DViewer;

// Preferences of the CAM workbench, where the simulator keeps its display options
ParameterGrp::handle CAMPreferences();

struct SimShape
{
public:
    float maxDimension() const;

public:
    std::vector<Vertex> verts;
    std::vector<GLushort> indices;
    bool needsUpdate = false;
};

struct SimTool
{
public:
    std::vector<float> profile;
    int id;
    float diameter;
    std::vector<Material::ProfileArc> arcs;
    double flutes = 0;
};

class DlgCAMSimulator: public QOpenGLWidget
{
    Q_OBJECT

    typedef std::chrono::steady_clock clock;

public:
    explicit DlgCAMSimulator(QWidget* parent = nullptr);
    ~DlgCAMSimulator() override;

    void connectTo(GuiDisplay& gui, Dummy3DViewer& dv);
    void cloneFrom(const DlgCAMSimulator& from);

    static DlgCAMSimulator* instance(Gui::Document* doc = nullptr);

    void setAnimating(bool animating);
    void startSimulation(const Part::TopoShape& stock, float quality);
    void resetSimulation();

    // A command of the toolpath, see GCodeParser::AddCommand. The commands are numbered in the
    // order they are added, from 0: the source of their motions.
    void addCommand(const std::string& name, const std::map<std::string, double>& params);
    // the commands added from now on belong to a new operation of this name, and of this color
    // if given (rgb from 0 to 1)
    void addOperation(const char* name, const float* color = nullptr);
    // arcs: of the profile, see Material::Tool
    void addTool(
        const std::vector<float>& toolProfilePoints,
        int toolNumber,
        float diameter,
        float resolution,
        const std::vector<Material::ProfileArc>& arcs = {},
        double flutes = 0
    );

    void setStockShape(const Part::TopoShape& shape, float resolution);
    void setStockVisible(bool b);
    void setBaseShape(const Part::TopoShape& shape, float resolution);
    void setBaseVisible(bool b);

    void setRotateEnabled(bool b);

    void setBackgroundColor(const QColor& c);
    void setPathColor(const QColor& normal, const QColor& rapid);
    void setStencilRendering(bool b);
    void setColorByOperation(bool b);
    void setThinWalls(bool enabled, float threshold);
    // color the stock by its distance from the model, within the tolerance green
    void setDeviation(bool enabled, float tolerance);
    // dark lines at the edges of the objects
    void setOutlines(bool enabled);
    // the origin of the job and its axes
    void setOriginVisible(bool visible);
    // lines over the stock to inspect it, see MillSimulation::SetInspect
    void setInspect(int mode);
    // opacity of the stock and the model, from 0 to 1
    void setOpacity(float stock, float model);

    // Frame timing for performance measurements. While enabled, every frame waits for the GPU to
    // finish, so the recorded times include the GPU work.
    void setFrameTiming(bool b);
    std::vector<double> takeFrameTimes();
    bool isRenderPending() const;
    std::map<std::string, std::string> rendererInfo() const;

    // Go to the start of the operation with this index (in the order they were added)
    void goToOperation(int index);
    // Pick the stock face at a position of the view (widget coordinates): the handler gets the
    // index of the operation that cut it, -1 for none, after the next frame
    void pickAt(const QPoint& pos);
    void setOperationPickedHandler(std::function<void(int)> handler);
    // The commands of the toolpath (numbered as they were added, see addCommand): go to the start
    // of a command, to the next or previous one (see MillSimulation::StepCommand), and the handler
    // gets the command in progress when it changes, -1 for none
    void goToCommand(int index);
    void stepCommand(int direction);
    void setCommandChangedHandler(std::function<void(int)> handler);
    // The problems found on the toolpath so far: go to the start of one, and the handler gets
    // their count when it changes
    std::vector<SimEvent> events() const;
    void goToEvent(int index);
    void setEventsChangedHandler(std::function<void(int)> handler);

Q_SIGNALS:
    void simulationStarted();
    void operationPicked(int operation);
    void commandChanged(int command);
    void eventsChanged(int count);

protected:
    void timerEvent(QTimerEvent* event) override;

    void updateResources();
    void updateWindowScale();
    void updateCamera();

    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

    void updateGui();

private:
    std::function<void(int)> mOperationPicked;
    std::function<void(int)> mCommandChanged;
    std::function<void(int)> mEventsChanged;
    int mEventCount = 0;  // the events shown
    void updateEvents();
    int mLastCommandShown = -2;  // the command in progress last reported
    bool mFrameTiming = false;
    std::vector<double> mFrameTimes;  // milliseconds
    std::string mGlVendor, mGlRenderer, mGlVersion;

    bool mNeedsInitialize = false;
    bool mNeedsClear = false;
    bool mAnimating = false;
    int mAnimatingTimer = 0;

    std::unique_ptr<MillSimulation> mMillSimulator;
    float mQuality = 10;

    // the toolpath: commands, and tool changes (source -1)
    struct Command
    {
        std::string name;
        std::map<std::string, double> params;
        int source;
    };
    std::vector<Command> mCommands;
    std::size_t mLastCommand = 0;
    int mSourceCount = 0;
    // operations: the index of their first command in mCommands, their name and color
    struct Operation
    {
        std::size_t firstCommand;
        std::string name;
        bool hasColor;
        float color[3];
    };
    std::vector<Operation> mOperations;
    std::size_t mLastOperation = 0;

    std::vector<SimTool> mTools;

    const SoCamera* mCamera = nullptr;
    SimShape mStock;
    SimShape mBase;
    // the model finely tessellated for the distance of the stock from it, made in another thread
    // while the distance is shown
    Part::TopoShape mBaseShape;
    bool mDeviation = false;
    std::future<std::shared_ptr<const Material::ModelDistance>> mDeviationModel;
    int mDeviationGeneration = 0;
    int mDeviationModelGeneration = -1;
    void startDeviationModel();

    std::unique_ptr<MillSimulationState> mState;
    clock::time_point mLastProcessSim = clock::time_point::min();

    GuiDisplay* mGui = nullptr;
    Dummy3DViewer* mDummyViewer = nullptr;
    const void* mLastCamera = nullptr;
    uint64_t mLastCameraId = 0;
};

}  // namespace CAMSimulator
