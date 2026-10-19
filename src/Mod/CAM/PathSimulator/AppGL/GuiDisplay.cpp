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

#include "GuiDisplay.h"

#include "ui_GuiDisplay.h"
#include <QCoreApplication>
#include <QLabel>
#include <QLinearGradient>
#include <QPainter>
#include <QSlider>
#include <QStyle>
#include <QStyleOptionSlider>
#include <cmath>
#include <limits>

// include this last as the defines can mess up other includes
#include "OpenGlWrapper.h"

namespace CAMSimulator
{

namespace
{

// The colors of the distance from the model, as the stock shader shows them (deviationColor in
// Shader.cpp): within the tolerance green, material left from light to dark blue up to 10
// tolerances, cut into the model from orange to dark red
class DeviationScale: public QWidget
{
public:
    explicit DeviationScale(QWidget* parent)
        : QWidget(parent)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        resize(190, 206);
    }
    void setTolerance(float tolerance)
    {
        mTolerance = tolerance;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        // readable on light and dark backgrounds
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(20, 20, 24, 170));
        p.drawRoundedRect(rect(), 6, 6);

        const int barX = 12, barW = 18, top = 40, height = 150;
        const int green = height / 5;  // the band within the tolerance
        const int half = (height - green) / 2;
        auto color = [](float r, float g, float b) {
            return QColor::fromRgbF(r, g, b);
        };
        QLinearGradient left(0, top, 0, top + half);
        left.setColorAt(0, color(0.05f, 0.15f, 0.6f));
        left.setColorAt(1, color(0.45f, 0.7f, 1.0f));
        p.fillRect(barX, top, barW, half, left);
        p.fillRect(barX, top + half, barW, green, color(0.25f, 0.8f, 0.3f));
        QLinearGradient cut(0, top + half + green, 0, top + height);
        cut.setColorAt(0, color(1.0f, 0.6f, 0.2f));
        cut.setColorAt(1, color(0.75f, 0.0f, 0.05f));
        p.fillRect(barX, top + half + green, barW, height - half - green, cut);

        p.setPen(Qt::white);
        QFont font = p.font();
        font.setBold(true);
        p.setFont(font);
        p.drawText(
            QRect(8, 4, width() - 16, 22),
            Qt::AlignLeft | Qt::AlignVCenter,
            QCoreApplication::translate("CAM_Simulator", "Distance from model")
        );
        font.setBold(false);
        p.setFont(font);
        const float t = mTolerance;
        auto mm = [](float v) {
            return QString::fromUtf8("%1%2 mm")
                .arg(
                    v > 0       ? "+"
                        : v < 0 ? "\u2212"
                                : ""
                )
                .arg(std::abs(v), 0, 'g', 3);
        };
        const int textX = barX + barW + 8;
        auto label = [&](int y, const QString& text) {
            p.drawLine(barX + barW, y, barX + barW + 4, y);
            p.drawText(
                QRect(textX, y - 9, width() - textX - 6, 18),
                Qt::AlignLeft | Qt::AlignVCenter,
                text
            );
        };
        label(
            top,
            QString::fromUtf8("\u2265 ") + mm(10 * t)
                + QCoreApplication::translate("CAM_Simulator", " left")
        );
        label(top + half, mm(t));
        label(top + half + green / 2, QCoreApplication::translate("CAM_Simulator", "on the model"));
        label(top + half + green, mm(-t));
        label(
            top + height,
            QString::fromUtf8("\u2264 ") + mm(-10 * t)
                + QCoreApplication::translate("CAM_Simulator", " cut in")
        );
    }

private:
    float mTolerance = 0.05f;
};

// Marks on the timeline slider, over it, at the problems found on the toolpath
class EventMarks: public QWidget
{
public:
    explicit EventMarks(QSlider* slider)
        : QWidget(slider)
        , mSlider(slider)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        slider->installEventFilter(this);
        setGeometry(slider->rect());
    }
    // the size of the slider
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (watched == mSlider && event->type() == QEvent::Resize) {
            setGeometry(mSlider->rect());
        }
        return false;
    }
    void setMarks(const std::vector<std::pair<float, QColor>>& marks)
    {
        mMarks = marks;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        if (mMarks.empty()) {
            return;
        }
        // where the slider puts its handle for a value, as its style draws it
        QStyleOptionSlider opt;
        opt.initFrom(mSlider);
        opt.orientation = mSlider->orientation();
        if (opt.orientation == Qt::Horizontal) {
            opt.state |= QStyle::State_Horizontal;  // as QSlider sets it
        }
        opt.minimum = 0;
        opt.maximum = 10000;
        const QStyle* style = mSlider->style();
        const QRect groove = style->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderGroove, mSlider);
        const QRect handle = style->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, mSlider);
        const int span = groove.width() - handle.width();
        QPainter p(this);
        for (const auto& [place, color] : mMarks) {
            const int x = groove.left() + handle.width() / 2
                + QStyle::sliderPositionFromValue(0, 10000, (int)(place * 10000), span);
            p.fillRect(QRect(x - 1, 1, 3, height() - 2), color);
        }
    }

private:
    QSlider* mSlider;
    std::vector<std::pair<float, QColor>> mMarks;
};

}  // namespace

GuiDisplay::GuiDisplay(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui_GuiDisplay)
{
    ui->setupUi(this);

    playing = true;
    setPlaying(false);

    speed = 0;
    setSpeed(10);

    connect(ui->slowerButton, &QToolButton::clicked, this, &GuiDisplay::onSlowerFasterButtonClicked);
    connect(ui->fasterButton, &QToolButton::clicked, this, &GuiDisplay::onSlowerFasterButtonClicked);
    connect(ui->viewAllButton, &QToolButton::clicked, this, &GuiDisplay::viewAll);
    connect(ui->rotateButton, &QToolButton::toggled, this, &GuiDisplay::rotateEnableChanged);
    connect(ui->pathButton, &QToolButton::toggled, this, &GuiDisplay::pathVisibleChanged);
    connect(ui->ssaoButton, &QToolButton::toggled, this, &GuiDisplay::ssaoEnableChanged);
    connect(ui->previousOperationButton, &QToolButton::clicked, this, &GuiDisplay::previousOperation);
    connect(ui->nextOperationButton, &QToolButton::clicked, this, &GuiDisplay::nextOperation);
    connect(ui->previousEventButton, &QToolButton::clicked, this, &GuiDisplay::previousEvent);
    connect(ui->nextEventButton, &QToolButton::clicked, this, &GuiDisplay::nextEvent);
    eventMarks = new EventMarks(ui->stageSlider);
    setEvents({});
    setOperation(QString(), -1, 0);
    connect(ui->stageSlider, &QSlider::sliderPressed, this, [this] { Q_EMIT scrubbing(true); });
    connect(ui->stageSlider, &QSlider::sliderReleased, this, [this] { Q_EMIT scrubbing(false); });

    deviationScale = new DeviationScale(this);
    deviationScale->hide();
    measureLabel = new QLabel(this);
    measureLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    measureLabel->setStyleSheet(
        QString::fromLatin1("color:white; background:rgba(20,20,24,170); padding:6px; border-radius:6px")
    );
    measureLabel->move(10, 10);
    measureLabel->hide();
}

void GuiDisplay::setDeviationScale(bool visible, float tolerance)
{
    static_cast<DeviationScale*>(deviationScale)->setTolerance(tolerance);
    deviationScale->setVisible(visible);
    placeDeviationScale();
}

void GuiDisplay::setMeasure(const QString& text)
{
    if (text == measureLabel->text() && measureLabel->isVisible() == !text.isEmpty()) {
        return;
    }
    measureLabel->setText(text);
    measureLabel->adjustSize();
    measureLabel->setVisible(!text.isEmpty());
    setMask(childrenRegion());
}

void GuiDisplay::setEvents(const std::vector<std::pair<float, QColor>>& marks)
{
    static_cast<EventMarks*>(eventMarks)->setMarks(marks);
    ui->previousEventButton->setVisible(!marks.empty());
    ui->nextEventButton->setVisible(!marks.empty());
}

// at the top right of the view
void GuiDisplay::placeDeviationScale()
{
    deviationScale->move(width() - deviationScale->width() - 10, 10);
    setMask(childrenRegion());
}

GuiDisplay::~GuiDisplay()
{
    delete ui;
}

void GuiDisplay::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    placeDeviationScale();
}

void GuiDisplay::setPlaying(bool b)
{
    if (b == playing) {
        return;
    }

    playing = b;

    const auto icon = playing ? ":/gl_simulator/Pause.png" : ":/gl_simulator/Play.png";
    ui->playButton->setIcon(QIcon(QString::fromUtf8(icon)));
}

void GuiDisplay::on_playButton_clicked()
{
    setPlaying(!playing);
    Q_EMIT play(playing);
}

void GuiDisplay::on_singleStepButton_clicked()
{
    setPlaying(false);
    Q_EMIT singleStep();
}

void GuiDisplay::on_skipStartButton_clicked()
{
    Q_EMIT skipToStart();
}

void GuiDisplay::on_skipEndButton_clicked()
{
    Q_EMIT skipToEnd();
}

void GuiDisplay::setSpeed(int s)
{
    if (s == speed) {
        return;
    }

    speed = s;
    ui->speedLabel->setText(tr("x%1").arg(speed));
}

// times the time of the machine
static const std::vector<int> speeds = {1, 2, 5, 10, 25, 50, 100, 250, 500, 1000};

std::vector<int>::const_iterator findNearestSpeed(int speed)
{
    int dist = std::numeric_limits<int>::max();
    auto ret = speeds.cend();

    for (auto it = speeds.cbegin(); it != speeds.cend(); it++) {
        const int curdist = std::abs(*it - speed);
        if (curdist < dist) {
            dist = curdist;
            ret = it;
        }
    }

    return ret;
}

void GuiDisplay::onSlowerFasterButtonClicked()
{
    auto it = findNearestSpeed(speed);

    const bool slower = sender() == ui->slowerButton;
    const bool faster = !slower;

    if (slower && it != speeds.begin()) {
        it--;
    }
    else if (faster && it != (speeds.end() - 1)) {
        it++;
    }

    setSpeed(*it);
    Q_EMIT speedChanged(*it);
}

void GuiDisplay::setStage(float f, int total)
{
    ui->stageSlider->setMaximum(total);
    ui->stageSlider->setValue(f * total);
}

// h:mm:ss, m:ss below an hour
static QString TimeText(double seconds)
{
    const long s = std::lround(std::max(0.0, seconds));
    if (s >= 3600) {
        return QString::fromLatin1("%1:%2:%3")
            .arg(s / 3600)
            .arg(s / 60 % 60, 2, 10, QLatin1Char('0'))
            .arg(s % 60, 2, 10, QLatin1Char('0'));
    }
    return QString::fromLatin1("%1:%2").arg(s / 60).arg(s % 60, 2, 10, QLatin1Char('0'));
}

void GuiDisplay::setTime(double current, double total)
{
    const QString text = total > 0 ? TimeText(current) + QString::fromLatin1(" / ") + TimeText(total)
                                   : QString();
    if (ui->timeLabel->text() != text) {
        ui->timeLabel->setText(text);
    }
}

void GuiDisplay::setOperation(const QString& name, int index, int count)
{
    // jumping between operations needs more than one
    ui->previousOperationButton->setVisible(count > 1);
    ui->nextOperationButton->setVisible(count > 1);
    const QString text = index >= 0 ? tr("%1 (%2 of %3)").arg(name).arg(index + 1).arg(count)
                                    : QString();
    if (ui->operationLabel->text() != text) {
        ui->operationLabel->setText(text);
    }
}

void GuiDisplay::on_stageSlider_sliderMoved(int value)
{
    const float f = (float)value / ui->stageSlider->maximum();
    Q_EMIT stageChanged(f);
}

void GuiDisplay::setRotateEnabled(bool b)
{
    ui->rotateButton->setChecked(b);
}

void GuiDisplay::setPathVisible(bool b)
{
    QSignalBlocker blocker(ui->pathButton);
    ui->pathButton->setChecked(b);
}

void GuiDisplay::setSsaoEnabled(bool b)
{
    QSignalBlocker blocker(ui->ssaoButton);
    ui->ssaoButton->setChecked(b);
}

}  // namespace CAMSimulator
