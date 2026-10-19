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

#include "PreCompiled.h"

#include "CAMSettings.h"

#include "DlgCAMSimulator.h"
#include <string_view>

using namespace std::literals;

namespace CAMSimulator
{

CAMSettings::CAMSettings(ParameterGrp::handle hGrp, DlgCAMSimulator& dlg)
    : hGrp(hGrp)
    , mDlg(dlg)
{
    hGrp->Attach(this);
}

CAMSettings::~CAMSettings()
{
    hGrp->Detach(this);
}

void CAMSettings::applySettings()
{
    OnChange(*hGrp, "DefaultNormalPathColor");
    OnChange(*hGrp, "DefaultRapidPathColor");
    OnChange(*hGrp, "SimulatorStencilRendering");
    OnChange(*hGrp, "SimulatorColorMode");
    OnChange(*hGrp, "SimulatorStockVisible");
    OnChange(*hGrp, "SimulatorInspect");
    OnChange(*hGrp, "SimulatorOutlines");
    OnChange(*hGrp, "SimulatorOriginVisible");
    OnChange(*hGrp, "SimulatorRapidRate");
    OnChange(*hGrp, "SimulatorStockOpacity");
}

void CAMSettings::OnChange(ParameterGrp::SubjectType& rCaller, ParameterGrp::MessageType Reason)
{
    (void)rCaller;

    if (Reason == "DefaultNormalPathColor"sv || Reason == "DefaultRapidPathColor"sv) {

        const unsigned long lcol
            = hGrp->GetUnsigned("DefaultNormalPathColor", 11141375UL);  // dark green (0,170,0)
        float lr, lg, lb;
        lr = ((lcol >> 24) & 0xff) / 255.0;
        lg = ((lcol >> 16) & 0xff) / 255.0;
        lb = ((lcol >> 8) & 0xff) / 255.0;

        const unsigned long rcol
            = hGrp->GetUnsigned("DefaultRapidPathColor", 2852126975UL);  // dark red (170,0,0)
        float rr, rg, rb;
        rr = ((rcol >> 24) & 0xff) / 255.0;
        rg = ((rcol >> 16) & 0xff) / 255.0;
        rb = ((rcol >> 8) & 0xff) / 255.0;

        mDlg.setPathColor(QColor::fromRgbF(lr, lg, lb), QColor::fromRgbF(rr, rg, rb));
    }
    else if (Reason == "SimulatorStencilRendering"sv) {
        mDlg.setStencilRendering(hGrp->GetBool("SimulatorStencilRendering", false));
    }
    else if (
        Reason == "SimulatorColorMode"sv || Reason == "SimulatorThinWallThreshold"sv
        || Reason == "SimulatorDeviationTolerance"sv
    ) {
        // the colors of the stock: plain, by operation, thin walls or by the distance from the
        // model, see SimulatorColorMode in the README
        const long mode = hGrp->GetInt("SimulatorColorMode", 1);
        mDlg.setColorByOperation(mode == 1);
        mDlg.setThinWalls(mode == 2, (float)hGrp->GetFloat("SimulatorThinWallThreshold", 0.5));
        mDlg.setDeviation(mode == 3, (float)hGrp->GetFloat("SimulatorDeviationTolerance", 0.05));
    }
    else if (Reason == "SimulatorOutlines"sv) {
        mDlg.setOutlines(hGrp->GetBool("SimulatorOutlines", true));
    }
    else if (Reason == "SimulatorRapidRate"sv) {
        // mm/min, of tools without a rapid rate
        mDlg.setRapidRate(hGrp->GetFloat("SimulatorRapidRate", 5000) / 60);
    }
    else if (Reason == "SimulatorOriginVisible"sv) {
        mDlg.setOriginVisible(hGrp->GetBool("SimulatorOriginVisible", true));
    }
    else if (Reason == "SimulatorInspect"sv) {
        mDlg.setInspect((int)hGrp->GetInt("SimulatorInspect", 0));
    }
    else if (Reason == "SimulatorStockVisible"sv || Reason == "SimulatorModelVisible"sv) {
        mDlg.setStockVisible(hGrp->GetBool("SimulatorStockVisible", true));
        mDlg.setBaseVisible(hGrp->GetBool("SimulatorModelVisible", false));
    }
    else if (Reason == "SimulatorStockOpacity"sv || Reason == "SimulatorModelOpacity"sv) {
        mDlg.setOpacity(
            (float)hGrp->GetFloat("SimulatorStockOpacity", 1.0),
            (float)hGrp->GetFloat("SimulatorModelOpacity", 1.0)
        );
    }
}

}  // namespace CAMSimulator
