# SPDX-License-Identifier: LGPL-2.1-or-later

# ***************************************************************************
# *   Copyright (c) 2017 Shai Seger <shaise at gmail>                       *
# *                                                                         *
# *   This program is free software; you can redistribute it and/or modify  *
# *   it under the terms of the GNU Lesser General Public License (LGPL)    *
# *   as published by the Free Software Foundation; either version 2 of     *
# *   the License, or (at your option) any later version.                   *
# *   for detail see the LICENCE text file.                                 *
# *                                                                         *
# *   This program is distributed in the hope that it will be useful,       *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
# *   GNU Library General Public License for more details.                  *
# *                                                                         *
# *   You should have received a copy of the GNU Library General Public     *
# *   License along with this program; if not, write to the Free Software   *
# *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
# *   USA                                                                   *
# *                                                                         *
# ***************************************************************************
"""
Command and task window handler for the OpenGL based CAM simulator
"""

import math
import os
import FreeCAD
import Path.Base.Util as PathUtil
import Path.Dressup.Utils as PathDressup
from Path.Main.SimulatorFeed import SimulatorFeed
from PathScripts import PathUtils
import CAMSimulator

from FreeCAD import Vector, Placement, Rotation

# lazily loaded modules
from lazy_loader.lazy_loader import LazyLoader

Mesh = LazyLoader("Mesh", globals(), "Mesh")
Part = LazyLoader("Part", globals(), "Part")

if FreeCAD.GuiUp:
    import FreeCADGui
    from PySide import QtGui, QtCore
    from PySide.QtGui import QDialogButtonBox

_filePath = os.path.dirname(os.path.abspath(__file__))


# Colors of the operations in the simulator when the stock is colored by operation, for operations
# without a path color of their own. Red shows thin walls.
OPERATION_COLORS = [
    (0.95, 0.60, 0.20),  # orange
    (0.45, 0.80, 0.35),  # green
    (0.95, 0.85, 0.30),  # yellow
    (0.70, 0.50, 0.90),  # purple
    (0.95, 0.50, 0.75),  # pink
    (0.40, 0.65, 0.95),  # blue
    (0.75, 0.60, 0.40),  # brown
    (0.70, 0.90, 0.35),  # lime
    (0.55, 0.85, 0.90),  # light blue
    (0.80, 0.80, 0.80),  # grey
]


def PathColor(op):
    """The color of the path of an operation, None if it has none"""
    view = getattr(op, "ViewObject", None)
    color = getattr(view, "NormalColor", None) if view else None
    return tuple(round(float(c), 3) for c in color[:3]) if color is not None else None


def OperationColors(ops):
    """Colors of operations in the simulator: their path colors where they have been set, otherwise
    colors of the palette. Path colors are default when they are those of the preferences or
    those of most operations, like after changing the preferences."""
    prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM")
    default = prefs.GetUnsigned("DefaultNormalPathColor", 11141375)
    defaults = [tuple(round(((default >> shift) & 0xFF) / 255.0, 3) for shift in (24, 16, 8))]
    colors = [PathColor(op) for op in ops]
    counts = {}
    for color in colors:
        counts[color] = counts.get(color, 0) + 1
    common = max(counts, key=counts.get) if counts else None
    if common is not None and counts[common] > 1:
        defaults.append(common)

    def isDefault(color):
        return color is None or any(
            all(abs(c - d) <= 1.5 / 255 for c, d in zip(color, other)) for other in defaults
        )

    return [
        OPERATION_COLORS[i % len(OPERATION_COLORS)] if isDefault(color) else color
        for i, color in enumerate(colors)
    ]


def IsSame(x, y):
    """Check if two floats are the same within an epsilon"""
    return abs(x - y) < 0.0001


def RadiusAt(edge, p):
    """Find the tool radius within a point on its circumference"""
    x = edge.valueAt(p).x
    y = edge.valueAt(p).y
    return math.sqrt(x * x + y * y)


class CAMSimTaskUi:
    """Handles the simulator task panel"""

    def __init__(self, parent):
        # this will create a Qt widget from our ui file
        self.form = FreeCADGui.PySideUic.loadUi(":/panels/TaskCAMSimulator.ui")
        self.parent = parent

    def getStandardButtons(self, *_args):
        """Task panel needs only Close button"""
        return QDialogButtonBox.Close

    def reject(self):
        """User Pressed the Close button"""
        self.parent.cancel()
        FreeCADGui.Control.closeDialog()


def TSError(msg):
    """Display error message"""
    QtGui.QMessageBox.information(None, "Path Simulation", msg)


class CAMSimulation:
    """Handles and prepares CAM jobs for simulation"""

    def __init__(self):
        self.debug = False
        self.stdrot = FreeCAD.Rotation(Vector(0, 0, 1), 0)
        self.iprogress = 0
        self.numCommands = 0
        self.simperiod = 20
        self.quality = 10
        self.resetSimulation = False
        self.jobs = []
        self.initdone = False
        self.taskForm = None
        self.disableAnim = False
        self.firstDrill = True
        self.millSim = None
        self.job = None
        self.activeOps = []
        self.ioperation = 0
        self.stock = None
        self.busy = False
        self.operations = []
        self.baseShape = None

    def Connect(self, but, sig):
        """Connect task panel buttons"""
        QtCore.QObject.connect(but, QtCore.SIGNAL("clicked()"), sig)

    def FindClosestEdge(self, edges, px, pz):
        """Convert tool shape to tool profile needed by GL simulator"""
        for edge in edges:
            p1 = edge.FirstParameter
            p2 = edge.LastParameter
            rad1 = RadiusAt(edge, p1)
            z1 = edge.valueAt(p1).z
            if IsSame(px, rad1) and IsSame(pz, z1):
                return edge, p1, p2
            rad2 = RadiusAt(edge, p2)
            z2 = edge.valueAt(p2).z
            if IsSame(px, rad2) and IsSame(pz, z2):
                return edge, p2, p1
            # sometimes a flat circle is without edge, so return edge with
            # same height and later a connecting edge will be interpolated
            if IsSame(pz, z1):
                return edge, p1, p2
            if IsSame(pz, z2):
                return edge, p2, p1
        return None, 0.0, 0.0

    def FindTopMostEdge(self, edges):
        """Examine tool solid edges and find the top most one"""
        maxz = -99999999.0
        topedge = None
        top_p1 = 0.0
        top_p2 = 0.0
        for edge in edges:
            p1 = edge.FirstParameter
            p2 = edge.LastParameter
            z = edge.valueAt(p1).z
            if z > maxz:
                topedge = edge
                top_p1 = p1
                top_p2 = p2
                maxz = z
            z = edge.valueAt(p2).z
            if z > maxz:
                topedge = edge
                top_p1 = p2
                top_p2 = p1
                maxz = z
        return topedge, top_p1, top_p2

    def GetToolProfile(self, tool, resolution):
        """Get the edge profile of a tool solid. Basically locating the
        side edge that OCC creates on any revolved object
        """
        shape = tool.Shape.copy()
        shape.Placement = Placement()
        sideEdgeList = []
        for _i, edge in enumerate(shape.Edges):
            if not edge.isClosed():
                # v1 = edge.firstVertex()
                # v2 = edge.lastVertex()
                # tp = "arc" if type(edge.Curve) is Part.Circle else "line"
                sideEdgeList.append(edge)

        # sort edges as a single 3d line on the x-z plane

        # first find the topmost edge
        edge, p1, p2 = self.FindTopMostEdge(sideEdgeList)
        profile = [RadiusAt(edge, p1), edge.valueAt(p1).z]
        endrad = 0.0
        # one by one find all connecting edges
        while edge is not None:
            sideEdgeList.remove(edge)
            if isinstance(edge.Curve, Part.Circle):
                # if edge is curved, approximate it with lines based on resolution, and with at
                # least 16 lines per quarter circle, so the arcs of small tools stay smooth
                nsegments = max(
                    int(edge.Length / resolution) + 1,
                    math.ceil(abs(p2 - p1) / (math.pi / 32)),
                )
                step = (p2 - p1) / nsegments
                location = p1 + step
                while nsegments > 0:
                    endrad = RadiusAt(edge, location)
                    endz = edge.valueAt(location).z
                    profile.append(endrad)
                    profile.append(endz)
                    location += step
                    nsegments -= 1
            else:
                endrad = RadiusAt(edge, p2)
                endz = edge.valueAt(p2).z
                profile.append(endrad)
                profile.append(endz)
            edge, p1, p2 = self.FindClosestEdge(sideEdgeList, endrad, endz)
            if edge is None:
                break
            startrad = RadiusAt(edge, p1)
            if not IsSame(startrad, endrad):
                profile.append(startrad)
                startz = edge.valueAt(p1).z
                profile.append(startz)

        return profile

    def Activate(self):
        """Invoke the simulator task panel"""
        self.initdone = False
        self.taskForm = CAMSimTaskUi(self)
        form = self.taskForm.form
        self.Connect(form.toolButtonPlay, self.SimPlay)
        form.sliderAccuracy.valueChanged.connect(self.onAccuracyBarChange)
        self.onAccuracyBarChange()

        prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM")
        if prefs.GetBool("SimulatorFollowsVisibility"):
            form.followsVisibility.setCheckState(QtCore.Qt.CheckState.Checked)
        form.followsVisibility.clicked.connect(self.followsVisibilityChange)

        # display options of the simulator view, preferences it follows
        threshold = prefs.GetFloat("SimulatorThinWallThreshold", 0.5)
        form.thinWallThreshold.setProperty("rawValue", threshold)
        form.thinWallThreshold.valueChanged.connect(self.onThinWallThresholdChange)
        form.stockOpacity.setValue(round(100 * prefs.GetFloat("SimulatorStockOpacity", 1.0)))
        form.modelOpacity.setValue(round(100 * prefs.GetFloat("SimulatorModelOpacity", 1.0)))
        form.stockOpacity.valueChanged.connect(self.onOpacityChange)
        form.modelOpacity.valueChanged.connect(self.onOpacityChange)

        self._populateJobSelection(form)
        form.comboJobs.currentIndexChanged.connect(self.onJobChange)
        self.onJobChange()
        form.listOperations.itemChanged.connect(self.onOperationItemChange)
        form.listOperations.itemClicked.connect(self.onOperationClicked)
        self.toggledItem = None
        FreeCADGui.Control.showDialog(self.taskForm)
        self.disableAnim = False
        self.firstDrill = True
        self.millSim = CAMSimulator.PathSim()
        self.initdone = True
        self.job = self.jobs[self.taskForm.form.comboJobs.currentIndex()]
        # self.SetupSimulation()

    def _populateJobSelection(self, form):
        """Make Job selection combobox"""
        # Get list of Job objects in active document
        jobList = FreeCAD.ActiveDocument.findObjects("Path::FeaturePython", "Job.*")

        # Get name of selected Job
        jobName = ""
        selection = FreeCADGui.Selection.getSelection()
        if selection:  #  Identify job selected by user
            job = PathUtils.findParentJob(selection[0])
            if job:
                jobName = job.Name

        # Prepare combobox
        form.comboJobs.blockSignals(True)
        form.comboJobs.clear()
        form.comboJobs.blockSignals(False)

        # Get index of selected Job
        setJobIdx = 0
        for i, job in enumerate(jobList):
            # Populate the job selection combobox
            form.comboJobs.addItem(job.ViewObject.Icon, job.Label)
            self.jobs.append(job)
            if job.Name == jobName:
                setJobIdx = i

        # Preselect GUI-selected job in the combobox
        form.comboJobs.setCurrentIndex(setJobIdx)

    def SetupSimulation(self):
        """Prepare all selected job operations for simulation"""
        form = self.taskForm.form
        self.activeOps = []
        self.numCommands = 0
        self.ioperation = 0
        for i in range(form.listOperations.count()):
            if form.listOperations.item(i).checkState() == QtCore.Qt.CheckState.Checked:
                self.firstDrill = True
                self.activeOps.append(self.operations[i])
                self.numCommands += len(self.operations[i].Path.Commands)

        self.stock = self.job.Stock.Shape
        self.busy = False

    def onJobChange(self):
        """When a new job is selected from the drop-down, update job operation list"""
        prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM")
        followsVisibility = prefs.GetBool("SimulatorFollowsVisibility")
        form = self.taskForm.form
        j = self.jobs[form.comboJobs.currentIndex()]
        self.job = j
        form.listOperations.clear()
        form.listOperations.setIconSize(QtCore.QSize(40, 16))
        self.operations = []
        activeOps = [op for op in j.Operations.OutList if PathUtil.opProperty(op, "Active")]
        self.operationColors = OperationColors(activeOps)
        allhidden = all(
            not op.Visibility for op in j.Operations.OutList if PathUtil.opProperty(op, "Active")
        )
        for op in j.Operations.OutList:
            if PathUtil.opProperty(op, "Active"):
                color = self.operationColors[len(self.operations)]
                listItem = QtGui.QListWidgetItem(
                    self._legendIcon(op.ViewObject.Icon, color), op.Label
                )
                listItem.setToolTip(
                    QtCore.QT_TRANSLATE_NOOP(
                        "CAM_Simulator",
                        "The color of the operation when the stock is colored by operation",
                    )
                )
                listItem.setFlags(listItem.flags() | QtCore.Qt.ItemIsUserCheckable)
                if followsVisibility and not op.Visibility and not allhidden:
                    listItem.setCheckState(QtCore.Qt.CheckState.Unchecked)
                else:
                    listItem.setCheckState(QtCore.Qt.CheckState.Checked)
                self.operations.append(op)
                form.listOperations.addItem(listItem)
        if len(j.Model.OutList) > 0:
            self.baseShape = Part.makeCompound([o.Shape for o in j.Model.OutList])
        else:
            self.baseShape = None
        self.toggledItem = None

    @staticmethod
    def _legendIcon(icon, color):
        """The icon of an operation with its color in the simulator beside it"""
        pixmap = QtGui.QPixmap(40, 16)
        pixmap.fill(QtCore.Qt.transparent)
        painter = QtGui.QPainter(pixmap)
        painter.drawPixmap(0, 0, icon.pixmap(16, 16))
        painter.setPen(QtGui.QColor(40, 40, 40))
        painter.setBrush(QtGui.QColor.fromRgbF(*color))
        painter.drawRect(22, 3, 14, 10)
        painter.end()
        return QtGui.QIcon(pixmap)

    def onAccuracyBarChange(self):
        """Update simulation quality"""
        form = self.taskForm.form
        self.quality = form.sliderAccuracy.value()
        qualText = QtCore.QT_TRANSLATE_NOOP("CAM_Simulator", "High")
        if self.quality < 4:
            qualText = QtCore.QT_TRANSLATE_NOOP("CAM_Simulator", "Low")
        elif self.quality < 9:
            qualText = QtCore.QT_TRANSLATE_NOOP("CAM_Simulator", "Medium")
        form.labelAccuracy.setText(qualText)

    def onThinWallThresholdChange(self, *_args):
        """Material thinner than the threshold is shown in red"""
        threshold = float(self.taskForm.form.thinWallThreshold.property("rawValue"))
        prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM")
        prefs.SetFloat("SimulatorThinWallThreshold", threshold)

    def onOpacityChange(self, *_args):
        """Opacity of the stock and the model in the simulator"""
        form = self.taskForm.form
        prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM")
        prefs.SetFloat("SimulatorStockOpacity", form.stockOpacity.value() / 100.0)
        prefs.SetFloat("SimulatorModelOpacity", form.modelOpacity.value() / 100.0)

    def followsVisibilityChange(self):
        """Update job list in accordance with operations visibility"""
        form = self.taskForm.form
        state = form.followsVisibility.isChecked()
        prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM")
        prefs.SetBool("SimulatorFollowsVisibility", state)
        self.onJobChange()

    def onOperationItemChange(self, item):
        """Check if at least one operation is selected to enable the Play button"""
        # a click on the check box toggles it, it does not go to the operation
        self.toggledItem = item
        playvalid = False
        form = self.taskForm.form
        for i in range(form.listOperations.count()):
            if form.listOperations.item(i).checkState() == QtCore.Qt.CheckState.Checked:
                playvalid = True
                break
        form.toolButtonPlay.setEnabled(playvalid)

    def onOperationClicked(self, item):
        """Go to the start of the clicked operation in the simulation"""
        if self.toggledItem is item:
            self.toggledItem = None
            return
        self.toggledItem = None
        if self.millSim is None:
            return
        op = self.operations[self.taskForm.form.listOperations.row(item)]
        if op in self.activeOps:
            self.millSim.GoToOperation(self.activeOps.index(op))

    def onOperationPicked(self, index):
        """A face of the stock was clicked in the simulator: select the operation that cut it"""
        listOperations = self.taskForm.form.listOperations
        if 0 <= index < len(self.activeOps) and self.activeOps[index] in self.operations:
            row = self.operations.index(self.activeOps[index])
            listOperations.setCurrentRow(row)
            listOperations.scrollToItem(listOperations.item(row))
        else:
            listOperations.clearSelection()
            listOperations.setCurrentItem(None)

    def SimulationSteps(self, ops, stock, colors):
        """SimulationSteps(ops, stock, colors) ... the calls that give ops to the
        simulator, in order: ("command", cmd), ("operation", label, color) and
        ("tool", profile, number, diameter). colors are those of ops."""
        feed = SimulatorFeed(stock.BoundBox)
        for op, color in zip(ops, colors):
            # an operation without a tool controller, a Custom one say, runs
            # with the tool already in the spindle
            tc = PathDressup.toolController(op)
            toolProfile = self.GetToolProfile(tc.Tool, 0.5) if tc is not None else None
            toolLength = max(toolProfile[1::2]) - min(toolProfile[1::2]) if toolProfile else None
            retract, opCommands = feed.add(op, toolLength)
            for cmd in retract:
                yield ("command", cmd)
            yield ("operation", op.Label, color)
            if tc is not None:
                yield ("tool", toolProfile, tc.ToolNumber, tc.Tool.Diameter)
            for cmd in opCommands:
                yield ("command", cmd)

    def SimPlay(self):
        """Activate the simulation"""
        self.SetupSimulation()
        self.millSim.ResetSimulation(FreeCADGui.getDocument(self.job.Document))
        colors = [self.operationColors[self.operations.index(op)] for op in self.activeOps]
        for step in self.SimulationSteps(self.activeOps, self.stock, colors):
            if step[0] == "command":
                self.millSim.AddCommand(step[1])
            elif step[0] == "operation":
                self.millSim.AddOperation(step[1], step[2])
            else:
                self.millSim.AddTool(step[1], step[2], step[3], 1)
        self.millSim.BeginSimulation(self.stock, self.quality)
        self.millSim.SetOperationPickedCallback(self.onOperationPicked)
        if self.baseShape is not None:
            self.millSim.SetBaseShape(self.baseShape, 1)

    def cancel(self):
        """Cancel the simulation"""
        if self.millSim is not None:
            self.millSim.SetOperationPickedCallback(None)


class CommandCAMSimulate:
    """FreeCAD invoke simulation task panel command"""

    def GetResources(self):
        """Command info"""
        return {
            "Pixmap": "CAM_SimulatorGL",
            "MenuText": QtCore.QT_TRANSLATE_NOOP("CAM_Simulator", "CAM Simulator"),
            "Accel": "P, N",
            "ToolTip": QtCore.QT_TRANSLATE_NOOP("CAM_Simulator", "Simulates G-code on stock"),
        }

    def IsActive(self):
        """Command is active if at least one CAM job exists"""
        if FreeCAD.ActiveDocument is not None:
            for o in FreeCAD.ActiveDocument.Objects:
                if o.Name[:3] == "Job":
                    return True
        return False

    def Activated(self):
        """Activate the simulation"""
        CamSimulation = CAMSimulation()
        CamSimulation.Activate()


if FreeCAD.GuiUp:
    # register the FreeCAD command
    FreeCADGui.addCommand("CAM_SimulatorGL", CommandCAMSimulate())
    FreeCAD.Console.PrintLog("Loading PathSimulator Gui… done\n")
