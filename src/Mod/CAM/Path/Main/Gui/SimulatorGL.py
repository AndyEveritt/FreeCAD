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

import os
import FreeCAD
import Path.Base.Util as PathUtil
from Path.Main.SimulatorFeed import commandText, eventText, simulationSteps, toolProfile
from PathScripts import PathUtils
import CAMSimulator

from FreeCAD import Vector

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


def TimeText(seconds):
    """h:mm:ss, m:ss below an hour"""
    s = int(round(max(0.0, seconds)))
    if s >= 3600:
        return "%d:%02d:%02d" % (s // 3600, s // 60 % 60, s % 60)
    return "%d:%02d" % (s // 60, s % 60)


# The colors of the kinds of problems, as the marks on the timeline of the simulator view
EVENT_COLORS = {
    "rapid": (255, 110, 20),
    "shank": (190, 80, 240),
    "gouge": (220, 30, 30),
    "left": (60, 120, 255),
}


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


if FreeCAD.GuiUp:

    class CommandListModel(QtCore.QAbstractListModel):
        """The commands the simulator runs, their text made when shown: there can be many."""

        def __init__(self, commands, operations, units):
            super().__init__()
            self.commands = commands  # (command, index of its operation)
            self.operations = operations  # labels
            self.units = units  # see commandText

        def rowCount(self, parent=QtCore.QModelIndex()):
            return 0 if parent.isValid() else len(self.commands)

        def data(self, index, role=QtCore.Qt.DisplayRole):
            if not index.isValid():
                return None
            cmd, op = self.commands[index.row()]
            if role == QtCore.Qt.DisplayRole:
                return "%d  %s" % (index.row() + 1, commandText(cmd, *self.units))
            if role == QtCore.Qt.ToolTipRole and 0 <= op < len(self.operations):
                return self.operations[op]
            return None


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

    def GetToolProfile(self, tool, resolution):
        """The profile of a tool solid, radius/height pairs from its top down to its tip"""
        return toolProfile(tool.Shape, resolution)[0]

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
        tolerance = prefs.GetFloat("SimulatorDeviationTolerance", 0.05)
        form.deviationTolerance.setProperty("rawValue", tolerance)
        form.deviationTolerance.valueChanged.connect(self.onDeviationToleranceChange)
        form.colorMode.setCurrentIndex(prefs.GetInt("SimulatorColorMode", 1))
        form.colorMode.currentIndexChanged.connect(self.onColorModeChange)
        self.onColorModeChange()
        form.inspect.setCurrentIndex(prefs.GetInt("SimulatorInspect", 0))
        form.inspect.currentIndexChanged.connect(self.onInspectChange)
        form.outlines.setChecked(prefs.GetBool("SimulatorOutlines", True))
        form.outlines.toggled.connect(self.onOutlinesChange)
        form.originVisible.setChecked(prefs.GetBool("SimulatorOriginVisible", True))
        form.originVisible.toggled.connect(self.onOriginChange)
        form.sectionAxis.setCurrentIndex(prefs.GetInt("SimulatorSection", 0))
        form.sectionPosition.setValue(round(1000 * prefs.GetFloat("SimulatorSectionPosition", 0.5)))
        form.sectionFlip.setChecked(prefs.GetBool("SimulatorSectionFlip", False))
        form.sectionAxis.currentIndexChanged.connect(self.onSectionChange)
        form.sectionPosition.valueChanged.connect(self.onSectionChange)
        form.sectionFlip.toggled.connect(self.onSectionChange)
        self.onSectionChange()
        form.stockVisible.setChecked(prefs.GetBool("SimulatorStockVisible", True))
        form.modelVisible.setChecked(prefs.GetBool("SimulatorModelVisible", False))
        form.stockVisible.toggled.connect(self.onVisibilityChange)
        form.modelVisible.toggled.connect(self.onVisibilityChange)
        form.stockOpacity.setValue(round(100 * prefs.GetFloat("SimulatorStockOpacity", 1.0)))
        form.modelOpacity.setValue(round(100 * prefs.GetFloat("SimulatorModelOpacity", 1.0)))
        form.stockOpacity.valueChanged.connect(self.onOpacityChange)
        form.modelOpacity.valueChanged.connect(self.onOpacityChange)

        self._populateJobSelection(form)
        form.comboJobs.currentIndexChanged.connect(self.onJobChange)
        self.onJobChange()
        form.listOperations.itemChanged.connect(self.onOperationItemChange)
        form.listOperations.itemClicked.connect(self.onOperationClicked)
        form.previousCommand.clicked.connect(lambda: self.onStepCommand(-1))
        form.nextCommand.clicked.connect(lambda: self.onStepCommand(1))
        form.listCommands.clicked.connect(self.onCommandClicked)
        form.listEvents.itemClicked.connect(self.onEventClicked)
        form.exportStock.clicked.connect(self.onExportStock)
        form.groupEvents.hide()
        font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        form.listCommands.setFont(font)
        self.commandModel = None
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

    def onColorModeChange(self, *_args):
        """The colors of the stock: plain, by operation, thin walls, or by the distance from the model"""
        form = self.taskForm.form
        mode = form.colorMode.currentIndex()
        form.thinWallThreshold.setEnabled(mode == 2)
        form.labelThinWallThreshold.setEnabled(mode == 2)
        form.deviationTolerance.setEnabled(mode == 3)
        form.labelDeviationTolerance.setEnabled(mode == 3)
        prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM")
        prefs.SetInt("SimulatorColorMode", mode)

    def onOutlinesChange(self, *_args):
        """Dark lines at the edges of the stock and the model"""
        prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM")
        prefs.SetBool("SimulatorOutlines", self.taskForm.form.outlines.isChecked())

    def onSectionChange(self, *_args):
        """A section of the stock and the model across an axis, where and which side"""
        form = self.taskForm.form
        prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM")
        axis = form.sectionAxis.currentIndex()
        form.sectionPosition.setEnabled(axis > 0)
        form.sectionFlip.setEnabled(axis > 0)
        prefs.SetFloat("SimulatorSectionPosition", form.sectionPosition.value() / 1000.0)
        prefs.SetBool("SimulatorSectionFlip", form.sectionFlip.isChecked())
        prefs.SetInt("SimulatorSection", axis)

    def onOriginChange(self, *_args):
        """The origin of the job and its axes in the simulator view"""
        prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM")
        prefs.SetBool("SimulatorOriginVisible", self.taskForm.form.originVisible.isChecked())

    def onInspectChange(self, *_args):
        """Lines over the stock to inspect it: its mesh, its normals or its dexels at a click"""
        prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM")
        prefs.SetInt("SimulatorInspect", self.taskForm.form.inspect.currentIndex())

    def onVisibilityChange(self, *_args):
        """Show the stock and the model in the simulator"""
        form = self.taskForm.form
        prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM")
        prefs.SetBool("SimulatorStockVisible", form.stockVisible.isChecked())
        prefs.SetBool("SimulatorModelVisible", form.modelVisible.isChecked())

    def onDeviationToleranceChange(self, *_args):
        """Faces within the tolerance of the model are green when colored by their distance"""
        tolerance = float(self.taskForm.form.deviationTolerance.property("rawValue"))
        prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM")
        prefs.SetFloat("SimulatorDeviationTolerance", tolerance)

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
        """A face of the stock was clicked in the simulator: select the operation that cut it and
        go to its start, like a click on it in the list"""
        listOperations = self.taskForm.form.listOperations
        if 0 <= index < len(self.activeOps) and self.activeOps[index] in self.operations:
            row = self.operations.index(self.activeOps[index])
            listOperations.setCurrentRow(row)
            listOperations.scrollToItem(listOperations.item(row))
            self.millSim.GoToOperation(index)
        else:
            listOperations.clearSelection()
            listOperations.setCurrentItem(None)

    def SimulationSteps(self, ops, stock, colors):
        """SimulationSteps(ops, stock, colors) ... see simulationSteps in SimulatorFeed"""
        return simulationSteps(ops, stock, colors)

    def SimPlay(self):
        """Activate the simulation"""
        self.SetupSimulation()
        self.millSim.ResetSimulation(FreeCADGui.getDocument(self.job.Document))
        colors = [self.operationColors[self.operations.index(op)] for op in self.activeOps]
        commands = []
        labels = []
        for step in self.SimulationSteps(self.activeOps, self.stock, colors):
            if step[0] == "command":
                self.millSim.AddCommand(step[1])
                commands.append((step[1], len(labels) - 1))
            elif step[0] == "operation":
                self.millSim.AddOperation(step[1], step[2])
                labels.append(step[1])
            else:
                self.millSim.AddTool(step[1], step[2], step[3], 1, **step[4])
        self.millSim.BeginSimulation(self.stock, self.quality)
        self.millSim.SetOperationPickedCallback(self.onOperationPicked)
        self.SetupCommands(commands, labels)
        if self.baseShape is not None:
            self.millSim.SetBaseShape(self.baseShape, 1)

    def SetupCommands(self, commands, labels):
        """The list of the commands the simulator runs, in the units of the preferences: those of
        the size of the stock, the same for all commands"""
        form = self.taskForm.form
        prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Units")
        decimals = prefs.GetInt("Decimals", 2)
        size = max(self.stock.BoundBox.DiagonalLength, 1.0)
        _, lengthFactor, lengthUnit = FreeCAD.Units.Quantity(size, FreeCAD.Units.Length).getUserPreferred()
        _, feedFactor, feedUnit = FreeCAD.Units.Quantity(
            1000.0 / 60, FreeCAD.Units.Velocity
        ).getUserPreferred()
        # the tool directions only if the tool turns
        rotary = any(abs(cmd.Parameters.get(k, 0)) > 1e-9 for cmd, _ in commands for k in "ABC")
        units = ((lengthFactor, max(decimals, 3)), (feedFactor, decimals), rotary)
        self.commandModel = CommandListModel(commands, labels, units)
        form.listCommands.setModel(self.commandModel)
        form.groupCommands.setTitle(
            QtCore.QT_TRANSLATE_NOOP("CAM_Simulator", "Commands") + " (%s, %s)" % (lengthUnit, feedUnit)
        )
        self.millSim.SetCommandChangedCallback(self.onCommandChanged)
        self.units = (lengthFactor, lengthUnit)
        self.operationLabels = labels
        self.taskForm.form.listEvents.clear()
        self.millSim.SetEventsChangedCallback(self.onEventsChanged)
        self.taskForm.form.cycleTime.setText("")
        self.taskForm.form.exportStock.setEnabled(True)
        self.showTimes(50)

    def onCommandChanged(self, index):
        """The simulation got to another command: show it in the list"""
        form = self.taskForm.form
        if self.commandModel is None:
            return
        count = self.commandModel.rowCount()
        if not 0 <= index < count:
            form.listCommands.clearSelection()
            form.commandLabel.setText("")
            return
        row = self.commandModel.index(index)
        form.listCommands.setCurrentIndex(row)
        form.listCommands.scrollTo(row, QtGui.QAbstractItemView.PositionAtCenter)
        _, op = self.commandModel.commands[index]
        label = self.commandModel.operations[op] if op >= 0 else ""
        form.commandLabel.setText("%d / %d  %s" % (index + 1, count, label))

    def showTimes(self, tries):
        """The time of the machine in all and of each operation, once the simulation has it"""
        if self.millSim is None:
            return
        times = self.millSim.GetTimes()
        if times["total"] <= 0:
            if tries > 0:
                QtCore.QTimer.singleShot(200, lambda: self.showTimes(tries - 1))
            return
        form = self.taskForm.form
        form.cycleTime.setText(
            QtCore.QT_TRANSLATE_NOOP("CAM_Simulator", "Cycle time") + " " + TimeText(times["total"])
        )
        for op, seconds in zip(self.activeOps, times["operations"]):
            if op in self.operations:
                item = form.listOperations.item(self.operations.index(op))
                item.setText("%s  (%s)" % (op.Label, TimeText(seconds)))

    def onExportStock(self):
        """Add the stock the simulator shows to the document of the job, as a mesh"""
        if self.millSim is None:
            return
        mesh = self.millSim.GetStockMesh()
        if mesh.CountFacets == 0:
            TSError(QtCore.QT_TRANSLATE_NOOP("CAM_Simulator", "The simulator shows no stock yet."))
            return
        doc = self.job.Document
        doc.openTransaction("Simulated stock")
        obj = doc.addObject("Mesh::Feature", "SimulatedStock")
        obj.Mesh = mesh
        obj.Label = "%s - %s" % (
            QtCore.QT_TRANSLATE_NOOP("CAM_Simulator", "Simulated stock"),
            self.job.Label,
        )
        doc.commitTransaction()
        doc.recompute()

    def onEventsChanged(self, count):
        """The simulation found problems on the toolpath: list them"""
        form = self.taskForm.form
        form.listEvents.clear()
        factor, unit = self.units
        for event in self.millSim.GetEvents():
            text = eventText(event, factor, unit)
            op = event["operation"]
            if 0 <= op < len(self.operationLabels):
                text = self.operationLabels[op] + ": " + text
            item = QtGui.QListWidgetItem(text)
            item.setIcon(self._eventIcon(event["kind"]))
            item.setToolTip(text)
            form.listEvents.addItem(item)
        form.groupEvents.setVisible(count > 0)
        form.groupEvents.setTitle(
            QtCore.QT_TRANSLATE_NOOP("CAM_Simulator", "Problems") + " (%d)" % count
        )

    @staticmethod
    def _eventIcon(kind):
        """A square in the color of the marks of a kind of problem on the timeline"""
        pixmap = QtGui.QPixmap(16, 16)
        pixmap.fill(QtCore.Qt.transparent)
        painter = QtGui.QPainter(pixmap)
        painter.setPen(QtGui.QColor(40, 40, 40))
        painter.setBrush(QtGui.QColor(*EVENT_COLORS.get(kind, (255, 255, 255))))
        painter.drawRect(2, 2, 11, 11)
        painter.end()
        return QtGui.QIcon(pixmap)

    def onEventClicked(self, item):
        """Go to the start of the clicked problem"""
        if self.millSim is not None:
            self.millSim.GoToEvent(self.taskForm.form.listEvents.row(item))

    def onCommandClicked(self, index):
        """Go to the start of the clicked command"""
        if self.millSim is not None and index.isValid():
            self.millSim.GoToCommand(index.row())

    def onStepCommand(self, direction):
        """Go to the next command, or to the start of the command in progress or the previous one"""
        if self.millSim is not None:
            self.millSim.StepCommand(direction)

    def cancel(self):
        """Cancel the simulation"""
        if self.millSim is not None:
            self.millSim.SetOperationPickedCallback(None)
            self.millSim.SetCommandChangedCallback(None)
            self.millSim.SetEventsChangedCallback(None)


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
