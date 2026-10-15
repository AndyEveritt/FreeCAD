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
        Add a path command to the simulation.
        """
        ...

    def AddOperation(self, name: str, /) -> None:
        """
        The commands and tools added from now on belong to a new operation of this name. The
        simulator shows the operation in progress and can go from one operation to the next.
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

    @no_args
    def GetRendererInfo(self) -> dict:
        """
        Return the OpenGL vendor, renderer and version and the simulator view size in pixels.
        """
        ...
