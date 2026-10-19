# SPDX-License-Identifier: LGPL-2.1-or-later

from __future__ import annotations

from typing import Any

from Base.BaseClass import BaseClass
from Base.Metadata import export, no_args

from Gui import Document
from Part.App.TopoShape import TopoShape
from CAM.App.Command import Command

@export(
    Include="Mod/CAM/PathSimulator/AppGL/CAMSim.h",
    FatherInclude="Base/BaseClassPy.h",
    Namespace="CAMSimulator",
    Constructor=True,
    Delete=True,
)
class CAMSim(BaseClass):
    """
    FreeCAD python wrapper of CAMSimulator

          CAMSimulator.CAMSim():

          Create a path simulator object

    Author: Shai Seger (shaise_at_g-mail)
    License: LGPL-2.1-or-later
    """

    def BeginSimulation(self, stock: TopoShape, resolution: float) -> None:
        """
        Start a simulation process on a box shape stock with given resolution
        """
        ...

    def ResetSimulation(self, document: Document, /) -> None:
        """
        Clear the simulation and all gcode commands
        """
        ...

    def AddTool(
        self,
        shape: list,
        toolnumber: int,
        diameter: float,
        resolution: float,
        arcs: list | None = None,
        flutes: float = 0,
        feeds: tuple = (0, 0),
        rapids: tuple = (0, 0),
    ) -> Any:
        """
        Set the shape of the tool to be used for simulation: its profile, radius/height pairs from
        its top down to its tip. arcs: the arcs of the profile, (first, last, r, z): the points
        first to last of the profile (indices of pairs) are on the circle around (r, z), the
        simulator follows them exactly. flutes: their length from the tip, 0 if unknown: the
        simulator finds the tool above them meeting the material. feeds and rapids: the
        (horizontal, vertical) feed rates of moves without a programmed one and rapid rates, mm/s,
        0 for unknown, for the time of the machine.
        """
        ...

    def SetBaseShape(self, shape: TopoShape, resolution: float) -> None:
        """
        Set the shape of the base object of the job
        """
        ...

    def AddCommand(self, command: Command, /) -> Any:
        """
        Add a path command to the simulation, in mm, feed rates in mm/s. The commands are
        numbered in the order they are added, from 0.
        """
        ...

    def AddOperation(self, name: str, color: tuple | None = None, /) -> None:
        """
        The commands and tools added from now on belong to a new operation of this name. The
        simulator shows the operation in progress and can go from one operation to the next.
        When the stock is colored by operation, the faces it cuts get the color (r, g, b) from 0
        to 1, or one of a palette without a color.
        """
        ...

    def SetFrameTiming(self, enabled: bool, /) -> None:
        """
        Enable or disable recording the time of each simulator frame, for performance
        measurements. While enabled, every frame waits for the GPU to finish.
        """
        ...

    @no_args
    def TakeFrameTimes(self) -> list:
        """
        Return the recorded frame times in milliseconds since the last call and clear them.
        """
        ...

    @no_args
    def IsRenderPending(self) -> bool:
        """
        Return True while the simulator view does not yet show the current simulation position,
        because the simulator spreads the work over several frames.
        """
        ...

    def GoToOperation(self, index: int, /) -> None:
        """
        Go to the start of an operation, by its index in the order the operations were added.
        """
        ...

    def SetOperationPickedCallback(self, callback: Any, /) -> None:
        """
        Call callback(index) when a face of the stock is clicked in the simulator view, with the
        index of the operation that cut it (in the order the operations were added), or -1 for
        none (uncut stock, or no stock there). None removes the callback.
        """
        ...

    def GoToCommand(self, index: int, /) -> None:
        """
        Go to the start of a command, by its index in the order the commands were added
        (AddCommand), or of the first command after it that moves the tool.
        """
        ...

    def StepCommand(self, direction: int, /) -> None:
        """
        Go to the start of the next command that moves the tool (direction > 0), or of the
        command in progress, or of the previous one if already there (direction < 0).
        """
        ...

    def SetCommandChangedCallback(self, callback: Any, /) -> None:
        """
        Call callback(index) when the command in progress changes, with its index in the order the
        commands were added, or -1 for none. None removes the callback.
        """
        ...

    @no_args
    def GetStockMesh(self) -> Any:
        """
        Return the stock the simulator shows, at the current position of the simulation, as a
        closed mesh (Mesh.Mesh), empty before it shows one.
        """
        ...

    @no_args
    def GetTimes(self) -> dict:
        """
        Return the time of the machine for the toolpath, s, once the simulation started: total,
        and operations, the time of each operation in the order they were added.
        """
        ...

    @no_args
    def GetEvents(self) -> list:
        """
        Return the problems found on the toolpath so far, in the order of the path, as dicts:
        kind ("rapid": a rapid move into the stock, "shank": the tool above its flutes meets the
        material, "gouge": a region of the stock cut into the model, "left": a region of faces cut
        away from the model), step (on the timeline: where it starts, where the operation that cut
        a region is done), command (the index of the command where it starts, -1 for none),
        operation (its index, -1 for none), volume (of the material met, mm3), position (its
        centroid, regions: of the deepest one), depth (regions: the largest distance from the
        model, mm), box (regions: (lo, hi)) and count (the regions of one kind and operation make
        one problem, their number).
        """
        ...

    def GoToEvent(self, index: int, /) -> None:
        """
        Go to the start of a problem, by its index in GetEvents.
        """
        ...

    def SetEventsChangedCallback(self, callback: Any, /) -> None:
        """
        Call callback(count) when the problems found on the toolpath change, with their count.
        None removes the callback.
        """
        ...

    @no_args
    def GetRendererInfo(self) -> dict:
        """
        Return the OpenGL vendor, renderer and version and the simulator view size in pixels.
        """
        ...
