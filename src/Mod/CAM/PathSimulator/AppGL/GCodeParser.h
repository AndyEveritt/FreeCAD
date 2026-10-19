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

#pragma once

#include "MillMotion.h"

#include <map>
#include <string>
#include <vector>

namespace CAMSimulator
{

// Turns a toolpath into motions: commands of a FreeCAD toolpath (AddCommand), or lines of G-code
// text (AddLine, Parse).
//
// Motion modes (G0 to G3, the drill cycles) are modal, like the feed rate (F), the arc plane
// (G17 to G19), absolute and incremental positions (G90, G91) and the units (G20, G21). Drill
// cycles (G73, G74, G76, G81 to G89) become the motions of the tool: to the hole at the height it
// starts from, rapid to R, down to Z and back to the initial height (G98) or R (G99). Arcs are
// given by their center (I, J, K) or their radius (R); arcs outside the XY plane are divided into
// lines.
class GCodeParser
{
public:
    GCodeParser() = default;
    virtual ~GCodeParser();

    void Clear();

    // A file of G-code text, false if it can't be read
    bool Parse(const char* filename);
    // A line of G-code text: feed rates in units per minute. Returns true if it moves.
    bool AddLine(const char* line, int source = -1);
    // A command of a FreeCAD toolpath: its name ("G1", "G81", "T3") and its words, in mm, feed
    // rates in mm/s. Returns true if it moves.
    bool AddCommand(
        const std::string& name,
        const std::map<std::string, double>& params,
        int source = -1
    );
    // The position before the first motions is unknown: the axes they don't name are taken from
    // the first motion that names them, so the tool starts where the program takes it, not at the
    // origin. Call once all motions are added.
    void FillStart();

public:
    std::vector<MillMotion> Operations;
    MillMotion lastState;
    // operation of the lines added from now on, see MillMotion::operation
    int currentOperation = -1;

    // Largest distance of the lines of arcs outside the XY plane from the arc, mm
    static constexpr double ArcTolerance = 0.001;

private:
    struct Word
    {
        char letter;
        double value;
    };
    static void Tokenize(const char* text, std::vector<Word>& words);
    bool Apply(const std::vector<Word>& words, double feedScale, int source);
    void AddArc(const MillMotion& end, bool clockwise, double ci, double cj, double ck);

    enum Mode
    {
        ModeNone,
        ModeRapid,
        ModeLine,
        ModeArcCW,
        ModeArcCCW,
        ModeCycle,
    };
    Mode mMode = ModeNone;
    int mPlane = 17;
    bool mIncremental = false;
    double mUnit = 1;  // mm per unit of the positions
    // drill cycles: the height the tool returns to with G98, R and the depth, modal
    bool mReturnToR = false;
    double mCycleInitial = 0;
    double mCycleR = 0;
    double mCycleDepth = 0;
};

}  // namespace CAMSimulator
