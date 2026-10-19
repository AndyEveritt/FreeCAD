// SPDX-License-Identifier: LGPL-2.1-or-later

// The machine of the CAM simulator: the tool directions its rotary axes reach, its travel

#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <Mod/CAM/PathSimulator/AppGL/MachineKinematics.h>

// NOLINTBEGIN(cppcoreguidelines-*,readability-*)

using namespace CAMSimulator;

namespace
{

MachineAxis Rotary(const char* name, double x, double y, double z, bool table, int sequence, double min = -360, double max = 360)
{
    MachineAxis a;
    a.name = name;
    a.vector[0] = x;
    a.vector[1] = y;
    a.vector[2] = z;
    a.table = table;
    a.sequence = sequence;
    a.min = min;
    a.max = max;
    return a;
}

MachineAxis Linear(const char* name, double min, double max)
{
    MachineAxis a;
    a.name = name;
    a.min = min;
    a.max = max;
    return a;
}

// the angles for random directions turn the tool along them
void ExpectRoundTrips(const MachineKinematics& m)
{
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> U(-1, 1);
    for (int i = 0; i < 200; i++) {
        double t[3] = {U(rng), U(rng), std::abs(U(rng)) + 0.05};
        const double n = std::sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
        for (double& v : t) {
            v /= n;
        }
        std::vector<double> angles;
        bool within;
        ASSERT_TRUE(m.AnglesFor(t, {}, angles, within)) << i;
        double back[3];
        m.ToolAxis(angles, back);
        for (int k = 0; k < 3; k++) {
            EXPECT_NEAR(back[k], t[k], 1e-9) << i << " " << k;
        }
    }
}

}  // namespace

TEST(SimulatorMachine, toolDirectionsOfTableAndHeadAxes)
{
    MachineKinematics tables;
    tables.SetAxes({}, {Rotary("A", 1, 0, 0, true, 1), Rotary("C", 0, 0, 1, true, 0)});
    ExpectRoundTrips(tables);
    MachineKinematics heads;
    heads.SetAxes({}, {Rotary("C", 0, 0, 1, false, 1), Rotary("B", 0, 1, 0, false, 0)});
    ExpectRoundTrips(heads);
    MachineKinematics mixed;
    mixed.SetAxes({}, {Rotary("B", 0, 1, 0, false, 0), Rotary("C", 0, 0, 1, true, 0)});
    ExpectRoundTrips(mixed);
}

TEST(SimulatorMachine, rotaryLimitsChooseTheSolution)
{
    // a C table carried by an A trunnion that tilts from -120 to 30 degrees
    MachineKinematics m;
    m.SetAxes({}, {Rotary("C", 0, 0, 1, true, 0), Rotary("A", 1, 0, 0, true, 1, -120, 30)});
    // the tool along -Y of the part: A -90
    std::vector<double> angles;
    bool within;
    double front[3] = {0, -1, 0};
    ASSERT_TRUE(m.AnglesFor(front, {}, angles, within));
    EXPECT_TRUE(within);
    EXPECT_NEAR(angles[1], -90, 1e-9);
    EXPECT_NEAR(std::remainder(angles[0], 360), 0, 1e-9);
    // along +Y: A +90 is beyond the limit, the table turns C half a turn instead
    double back[3] = {0, 1, 0};
    ASSERT_TRUE(m.AnglesFor(back, {}, angles, within));
    EXPECT_TRUE(within);
    EXPECT_NEAR(angles[1], -90, 1e-9);
    EXPECT_NEAR(std::abs(std::remainder(angles[0], 360)), 180, 1e-9);
    // from below the part: A would turn 180 degrees
    double below[3] = {0, 0, -1};
    ASSERT_TRUE(m.AnglesFor(below, {}, angles, within));
    EXPECT_FALSE(within);

    // a machine without rotary axes reaches the vertical only
    MachineKinematics three;
    three.SetAxes({Linear("X", 0, 100)}, {});
    double z[3] = {0, 0, 1};
    EXPECT_TRUE(three.AnglesFor(z, {}, angles, within));
    EXPECT_FALSE(three.AnglesFor(front, {}, angles, within));
}

TEST(SimulatorMachine, eventsOfTheLimitsOfTheMachine)
{
    // 3 axes, X travels 100 mm: moves with the tool tilted, a toolpath 105 mm wide. The moves
    // turning the tool are left out, the machine turns its own way.
    MachineKinematics m;
    m.SetAxes({Linear("X", 0, 100), Linear("Y", 0, 100), Linear("Z", -50, 50)}, {});
    MillMotion start;
    start.x = -10;
    start.operation = 0;
    std::vector<MillMotion> ends(5, start);
    const double xs[5] = {0, 5, 10, 12, 95};
    const double as[5] = {0, 90, 90, 0, 0};  // A 90: the tool along -Y
    for (int i = 0; i < 5; i++) {
        ends[i].x = xs[i];
        ends[i].a = as[i];
        ends[i].operation = i < 1 ? 0 : 1;
    }
    const auto events = MachineEvents(m, start, ends);
    ASSERT_EQ(events.size(), 2u);
    EXPECT_EQ(events[0].kind, Material::Event::MachineLimit);
    EXPECT_TRUE(events[0].axis.empty());
    EXPECT_EQ(events[0].firstSegment, 2);
    EXPECT_EQ(events[0].operation, 1);
    EXPECT_NEAR(events[0].direction[1], -1, 1e-9);
    EXPECT_EQ(events[1].axis, "X");
    EXPECT_EQ(events[1].firstSegment, 4);
    EXPECT_NEAR(events[1].needed, 105, 1e-9);
    EXPECT_NEAR(events[1].available, 100, 1e-9);
    // an unknown machine finds nothing
    EXPECT_TRUE(MachineEvents(MachineKinematics(), start, ends).empty());
}

// NOLINTEND(cppcoreguidelines-*,readability-*)
