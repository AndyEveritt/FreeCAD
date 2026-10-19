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

    def AddTool(self, shape: TopoShape, toolnumber: int, diameter: float, resolution: float) -> Any:
        """
        Set the shape of the tool to be used for simulation
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

    @no_args
    def GetRendererInfo(self) -> dict:
        """
        Return the OpenGL vendor, renderer and version and the simulator view size in pixels.
        """
        ...
