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

#include <QColor>
#include <QWidget>

#include <utility>
#include <vector>

namespace CAMSimulator
{
class Ui_GuiDisplay;

class GuiDisplay: public QWidget
{
    Q_OBJECT

public:
    explicit GuiDisplay(QWidget* parent = nullptr);
    ~GuiDisplay();

    void setPlaying(bool b);
    void setSpeed(int s);
    void setStage(float f, int total);
    // the time of the machine at the tool and in all, s
    void setTime(double current, double total);
    // the operation in progress, index -1 for none
    void setOperation(const QString& name, int index, int count);
    // the scale of the colors of the distance from the model, while they are shown
    void setDeviationScale(bool visible, float tolerance);
    // marks on the timeline at the problems found on the toolpath: their place from 0 to 1 and
    // their color
    void setEvents(const std::vector<std::pair<float, QColor>>& marks);

    void setRotateEnabled(bool b);
    void setPathVisible(bool b);
    void setSsaoEnabled(bool b);

Q_SIGNALS:
    void play(bool b);
    void singleStep();
    void skipToStart();
    void skipToEnd();
    void previousOperation();
    void nextOperation();
    void previousEvent();
    void nextEvent();
    void speedChanged(int s);
    void stageChanged(float f);
    void scrubbing(bool b);

    void viewAll();
    void rotateEnableChanged(bool b);
    void pathVisibleChanged(bool b);
    void ssaoEnableChanged(bool b);

protected:
    void resizeEvent(QResizeEvent* event) override;

private Q_SLOTS:
    void on_playButton_clicked();
    void on_singleStepButton_clicked();
    void on_skipStartButton_clicked();
    void on_skipEndButton_clicked();

    void onSlowerFasterButtonClicked();
    void on_stageSlider_sliderMoved(int value);


private:
    void placeDeviationScale();
    Ui_GuiDisplay* ui;
    QWidget* deviationScale = nullptr;
    QWidget* eventMarks = nullptr;

    bool playing = true;
    int speed = 1;
};

}  // namespace CAMSimulator
