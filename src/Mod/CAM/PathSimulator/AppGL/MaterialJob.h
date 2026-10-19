// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2026 The FreeCAD project association AISBL              *
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

#include "MaterialSimulation.h"

#include <cstdint>
#include <string>
#include <vector>

namespace CAMSimulator::Material
{

// The input of a simulation: the tools (profiles as for Tool), the toolpath and the stock mesh. It
// is written to a file to replay the simulation outside of the GUI, for the accuracy validation
// and for debugging (the environment variable CAM_SIM_DUMP).
struct Job
{
    float quality = 10;
    std::vector<std::vector<float>> profiles;
    std::vector<std::vector<ProfileArc>> arcs;  // of the profiles, empty or one list per profile
    std::vector<Segment> segments;
    std::vector<float> stockXyz;
    std::vector<uint32_t> stockTriangles;
};

// Write the job to a file, false if it can't be written
bool SaveJob(const std::string& path, const Job& job);
// Read a job written by SaveJob, false if the file can't be read or isn't a job
bool LoadJob(const std::string& path, Job& job);

}  // namespace CAMSimulator::Material
