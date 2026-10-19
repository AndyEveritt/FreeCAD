// SPDX-License-Identifier: LGPL-2.1-or-later

// The toolpath of the CAM simulator: commands and G-code text into motions

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <Mod/CAM/PathSimulator/AppGL/GCodeParser.h>
#include <Mod/CAM/PathSimulator/AppGL/MachineKinematics.h>
#include <Mod/CAM/PathSimulator/AppGL/ToolpathCheck.h>

// NOLINTBEGIN(cppcoreguidelines-*,readability-*)

using namespace CAMSimulator;

namespace
{

std::vector<MillMotion> Lines(const std::vector<std::string>& lines)
{
    GCodeParser parser;
    for (size_t i = 0; i < lines.size(); i++) {
        parser.AddLine(lines[i].c_str(), (int)i);
    }
    return parser.Operations;
}

}  // namespace

TEST(SimulatorGCode, commandsKeepRapidsFeedsAndTheirSource)
{
    GCodeParser parser;
    parser.AddCommand("G0", {{"X", 1}, {"Y", 2}, {"Z", 3}}, 0);
    parser.AddCommand("G1", {{"Z", -1}, {"F", 5}}, 1);
    parser.AddCommand("G1", {{"X", 10}}, 2);
    parser.AddCommand("T", {{"T", 4}}, 3);
    parser.AddCommand("G0", {{"Z", 3}}, 4);
    const auto& m = parser.Operations;
    ASSERT_EQ(m.size(), 4u);
    EXPECT_TRUE(m[0].rapid);
    EXPECT_FALSE(m[1].rapid);
    EXPECT_DOUBLE_EQ(m[1].feed, 5);  // mm/s, as the command has it
    EXPECT_DOUBLE_EQ(m[2].feed, 5);  // modal
    EXPECT_DOUBLE_EQ(m[2].x, 10);
    EXPECT_DOUBLE_EQ(m[2].y, 2);
    EXPECT_DOUBLE_EQ(m[2].z, -1);
    EXPECT_EQ(m[2].source, 2);
    EXPECT_TRUE(m[3].rapid);
    EXPECT_EQ(m[3].tool, 4);
    EXPECT_EQ(m[3].source, 4);
}

TEST(SimulatorGCode, positionsInDoublePrecision)
{
    GCodeParser parser;
    parser.AddCommand("G1", {{"X", 1234.0000005}, {"Y", -0.1}}, 0);
    EXPECT_DOUBLE_EQ(parser.Operations[0].x, 1234.0000005);
    const auto m = Lines({"G1 X1234.0000005 Y-.1"});
    EXPECT_NEAR(m[0].x, 1234.0000005, 1e-10);
    EXPECT_NEAR(m[0].y, -0.1, 1e-15);
}

TEST(SimulatorGCode, textIsReadWordByWord)
{
    // modal motion mode, feeds per minute, comments within lines, words without spaces (not
    // hexadecimal numbers), line numbers
    const auto m = Lines({
        "N10 G0X1Y2Z3",
        "G1 (go down) Z-1 F600 ; comment X99",
        "X5",
        "(only a comment X7)",
        "G02 X5 Y8 I0 J3",
    });
    ASSERT_EQ(m.size(), 4u);
    EXPECT_DOUBLE_EQ(m[0].x, 1);
    EXPECT_DOUBLE_EQ(m[0].y, 2);
    EXPECT_TRUE(m[0].rapid);
    EXPECT_DOUBLE_EQ(m[1].z, -1);
    EXPECT_DOUBLE_EQ(m[1].x, 1);
    EXPECT_DOUBLE_EQ(m[1].feed, 10);
    EXPECT_FALSE(m[2].rapid);
    EXPECT_DOUBLE_EQ(m[2].x, 5);
    EXPECT_EQ(m[2].source, 2);
    EXPECT_EQ(m[3].cmd, eRotateCW);
    EXPECT_DOUBLE_EQ(m[3].j, 3);
}

TEST(SimulatorGCode, incrementalPositionsAndInches)
{
    const auto m = Lines({"G1 X1 Y1 Z1", "G91 X1 Y-2", "G90 G20 X1 F10", "G21 Z2"});
    ASSERT_EQ(m.size(), 4u);
    EXPECT_DOUBLE_EQ(m[1].x, 2);
    EXPECT_DOUBLE_EQ(m[1].y, -1);
    EXPECT_DOUBLE_EQ(m[2].x, 25.4);
    EXPECT_DOUBLE_EQ(m[2].feed, 25.4 * 10 / 60);
    EXPECT_DOUBLE_EQ(m[3].z, 2);
}

TEST(SimulatorGCode, homeAndOffsetsAreNotMoves)
{
    const auto m = Lines({"G0 X1 Y1 Z5", "G28 G91 Z0", "G92 X0 Y0", "G90 G0 X2"});
    ASSERT_EQ(m.size(), 2u);
    EXPECT_DOUBLE_EQ(m[1].x, 2);
    EXPECT_DOUBLE_EQ(m[1].z, 5);
}

TEST(SimulatorGCode, drillCyclesAreTheMovesOfTheTool)
{
    // G98: back to the height of the start, the cycle is modal with its depth and R
    const auto m = Lines({"G0 X0 Y0 Z10", "G98 G81 X5 Y5 Z-3 R2 F60", "X8", "G80 G0 Z20"});
    // to the hole at 10, down to R, to the depth, back to 10; again at X8, then the rapid
    ASSERT_EQ(m.size(), 1u + 4 + 4 + 1);
    const double expected[][4] = {
        {5, 5, 10, 1},
        {5, 5, 2, 1},
        {5, 5, -3, 0},
        {5, 5, 10, 1},
        {8, 5, 10, 1},
        {8, 5, 2, 1},
        {8, 5, -3, 0},
        {8, 5, 10, 1},
    };
    for (int i = 0; i < 8; i++) {
        EXPECT_DOUBLE_EQ(m[i + 1].x, expected[i][0]) << i;
        EXPECT_DOUBLE_EQ(m[i + 1].y, expected[i][1]) << i;
        EXPECT_DOUBLE_EQ(m[i + 1].z, expected[i][2]) << i;
        EXPECT_EQ(m[i + 1].rapid, expected[i][3] != 0) << i;
    }
    EXPECT_EQ(m[5].source, 2);
    EXPECT_DOUBLE_EQ(m[9].z, 20);

    // G99: back to R, below a start lower than R the tool first goes up
    const auto r = Lines({"G0 X0 Y0 Z1", "G99 G83 X5 Y5 Z-3 R2 Q1"});
    ASSERT_EQ(r.size(), 1u + 5);
    EXPECT_DOUBLE_EQ(r[1].x, 0);
    EXPECT_DOUBLE_EQ(r[1].z, 2);
    EXPECT_DOUBLE_EQ(r[5].z, 2);
}

TEST(SimulatorGCode, arcsByTheirRadius)
{
    // a quarter circle around the origin, counterclockwise from (10, 0) to (0, 10)
    auto m = Lines({"G0 X10 Y0", "G3 X0 Y10 R10"});
    ASSERT_EQ(m.size(), 2u);
    EXPECT_EQ(m[1].cmd, eRotateCCW);
    EXPECT_NEAR(m[1].i, -10, 1e-12);
    EXPECT_NEAR(m[1].j, 0, 1e-12);
    // clockwise, the center on the other side
    m = Lines({"G0 X10 Y0", "G2 X0 Y10 R10"});
    EXPECT_NEAR(m[1].i, 0, 1e-12);
    EXPECT_NEAR(m[1].j, 10, 1e-12);
    // more than a half circle by a negative radius
    m = Lines({"G0 X10 Y0", "G3 X0 Y10 R-10"});
    EXPECT_NEAR(m[1].i, 0, 1e-12);
    EXPECT_NEAR(m[1].j, 10, 1e-12);
}

TEST(SimulatorGCode, arcsInEveryPlane)
{
    // G18: a half circle in the XZ plane around (0, 0, 0), from X10 to X-10, one motion; clockwise
    // seen from +Y goes from +X through +Z
    const auto m = Lines({"G0 X10 Y3 Z0", "G18 G2 X-10 Z0 I-10 K0"});
    ASSERT_EQ(m.size(), 2u);
    EXPECT_EQ(m[1].cmd, eRotateCW);
    EXPECT_EQ(m[1].plane, 18);
    const Material::Segment arc = MotionSegment(m[0], m[1]);
    ASSERT_TRUE(arc.arc);
    EXPECT_NEAR(std::abs(arc.sweep), M_PI, 1e-9);
    for (int i = 0; i <= 10; i++) {
        double p[3];
        arc.PositionAt(i / 10.0, p);
        EXPECT_NEAR(std::hypot(p[0], p[2]), 10, 1e-9) << i;
        EXPECT_NEAR(p[1], 3, 1e-12) << i;
        EXPECT_GE(p[2], -1e-9) << i;
    }
    double end[3];
    arc.PositionAt(1, end);
    EXPECT_NEAR(end[0], -10, 1e-9);
    // G19 in the YZ plane, counterclockwise seen from +X: from +Y towards +Z, a helix along X
    const auto n = Lines({"G0 X0 Y5 Z0", "G19 G3 X4 Y-5 Z0 J-5 K0"});
    ASSERT_EQ(n.size(), 2u);
    const Material::Segment helix = MotionSegment(n[0], n[1]);
    double mid[3];
    helix.PositionAt(0.5, mid);
    EXPECT_NEAR(mid[0], 2, 1e-9);
    EXPECT_NEAR(mid[1], 0, 1e-9);
    EXPECT_NEAR(mid[2], 5, 1e-9);
    // its time: the arc of radius 5, half a circle, at 10 mm/s
    MillMotion feed = n[1];
    feed.feed = 10;
    EXPECT_NEAR(MotionSeconds(n[0], feed, helix, ToolRates(), 50), std::hypot(5 * M_PI, 4) / 10, 1e-9);
}

TEST(SimulatorGCode, theToolStartsWhereTheProgramTakesIt)
{
    // not at the origin: the first motions take the axes they don't name from the first motion
    // that names them
    GCodeParser parser;
    for (const char* line : {"G0 Z15", "G0 X30", "G0 Y20", "G1 Z-1", "G1 X40"}) {
        parser.AddLine(line);
    }
    parser.FillStart();
    const auto& m = parser.Operations;
    ASSERT_EQ(m.size(), 5u);
    for (int i = 0; i < 3; i++) {
        EXPECT_DOUBLE_EQ(m[i].x, 30) << i;
        EXPECT_DOUBLE_EQ(m[i].y, 20) << i;
        EXPECT_DOUBLE_EQ(m[i].z, 15) << i;
    }
    EXPECT_DOUBLE_EQ(m[3].z, -1);
    EXPECT_DOUBLE_EQ(m[4].x, 40);
    // axes never named stay at 0
    EXPECT_DOUBLE_EQ(m[0].a, 0);
}

TEST(SimulatorGCode, theTimeOfTheMachine)
{
    // F600 mm/min: 10 mm/s
    GCodeParser parser;
    for (const char* line : {"G0 X0 Y0 Z10", "G1 X30 Y40 F600", "G0 X0 Y0 Z40", "G1 Z0", "G2 X20 Y0 I10 J0"}) {
        parser.AddLine(line);
    }
    const auto& m = parser.Operations;
    ToolRates rates;
    rates.horizFeed = 5;
    rates.vertFeed = 2;
    rates.horizRapid = 100;
    rates.vertRapid = 20;
    auto seconds = [&](int i) {
        return MotionSeconds(m[i - 1], m[i], MotionSegment(m[i - 1], m[i]), rates, 50);
    };
    // 50 mm at the programmed feed rate
    EXPECT_NEAR(seconds(1), 5, 1e-9);
    // rapid: 50 mm across at 100 mm/s, 30 mm up at 20 mm/s, the axes at once
    EXPECT_NEAR(seconds(2), 1.5, 1e-9);
    // feed rates are modal
    EXPECT_NEAR(seconds(3), 4, 1e-9);
    // a half circle of radius 10
    EXPECT_NEAR(seconds(4), M_PI, 1e-9);
    // without a programmed feed rate: the vertical one of the tool for a plunge, the horizontal
    // one otherwise; without rapid rates the default
    GCodeParser bare;
    for (const char* line : {"G0 X0 Y0 Z10", "G1 Z0", "G1 X10", "G0 X20"}) {
        bare.AddLine(line);
    }
    const auto& b = bare.Operations;
    auto bareSeconds = [&](int i, const ToolRates& r) {
        return MotionSeconds(b[i - 1], b[i], MotionSegment(b[i - 1], b[i]), r, 50);
    };
    EXPECT_NEAR(bareSeconds(1, rates), 5, 1e-9);
    EXPECT_NEAR(bareSeconds(2, rates), 2, 1e-9);
    EXPECT_NEAR(bareSeconds(3, ToolRates()), 0.2, 1e-9);
}

TEST(SimulatorGCode, postedTiltedWorkPlanes)
{
    // the back chamfer of the manifold demo: G68.2 with the origin on the chamfer and its normal
    // (0, 1, 1) / sqrt(2) as Rz(180) Rx(45)
    const double s = std::sqrt(0.5);
    const auto m = Lines({
        "G0 X10 Y10 Z50",
        "G68.2 X70 Y82.5 Z37.5 I180 J45 K0",
        "G53.1",
        "G0 X0 Y0 Z10",
        "G1 Z0 F600",
        "G2 X10 Y0 I5 J0",
        "G69",
        "G0 X1 Y2 Z60",
        "G38.2 Z40 F100",
    });
    ASSERT_GT(m.size(), 6u);
    // in the plane: the part at the origin plus R p, the tool along the normal of the chamfer
    EXPECT_NEAR(m[1].x, 70, 1e-9);
    EXPECT_NEAR(m[1].y, 82.5 + 10 * s, 1e-9);
    EXPECT_NEAR(m[1].z, 37.5 + 10 * s, 1e-9);
    double axis[3];
    MotionToolAxis(m[1], axis);
    EXPECT_NEAR(axis[0], 0, 1e-9);
    EXPECT_NEAR(axis[1], s, 1e-9);
    EXPECT_NEAR(axis[2], s, 1e-9);
    // the arc of the plane, in lines on its circle in the chamfer: X of the plane is -X of the part
    const size_t arcEnd = m.size() - 3;
    for (size_t i = 3; i <= arcEnd; i++) {
        // on the plane: along its normal (0, s, s) at the origin; Y of the plane is (0, -s, s)
        const double dy = m[i].y - 82.5, dz = m[i].z - 37.5;
        EXPECT_NEAR(dy + dz, 0, 1e-9) << i;
        EXPECT_NEAR(std::hypot(m[i].x - (70 - 5), (dz - dy) * s), 5, 1e-6) << i;
    }
    EXPECT_NEAR(m[arcEnd].x, 60, 1e-9);
    // G69: back in the part, the tool vertical; probing is a feed move
    EXPECT_NEAR(m[m.size() - 2].x, 1, 1e-12);
    EXPECT_NEAR(m[m.size() - 2].z, 60, 1e-12);
    MotionToolAxis(m.back(), axis);
    EXPECT_NEAR(axis[2], 1, 1e-12);
    EXPECT_FALSE(m.back().rapid);
}

TEST(SimulatorGCode, theControlTakesThePositionIntoANewPlane)
{
    // a move naming Z only after G68.2 goes along the normal of the plane from where the tool is,
    // and after G69 from where it is in the part
    const double s = std::sqrt(0.5);
    GCodeParser parser;
    for (const char* line : {"G0 X10 Y10 Z50", "G68.2 X70 Y82.5 Z37.5 I180 J45 K0", "G0 Z30", "G69", "G0 Z60"}) {
        parser.AddLine(line);
    }
    const auto& m = parser.Operations;
    ASSERT_EQ(m.size(), 3u);
    // the plane height of (10, 10, 50): its distance along the normal (0, s, s) from the origin
    const double height = (10 - 82.5) * s + (50 - 37.5) * s;
    EXPECT_NEAR(m[1].x, 10, 1e-9);
    EXPECT_NEAR(m[1].y, 10 + (30 - height) * s, 1e-9);
    EXPECT_NEAR(m[1].z, 50 + (30 - height) * s, 1e-9);
    EXPECT_NEAR(m[2].x, m[1].x, 1e-9);
    EXPECT_NEAR(m[2].y, m[1].y, 1e-9);
    EXPECT_NEAR(m[2].z, 60, 1e-9);
}

TEST(SimulatorGCode, postedRotaryWordsAreTheAnglesOfTheMachine)
{
    // a C table carried by an A trunnion: A-90 turns the part so the tool is along its -Y, the
    // program positions in the frame the table turned it to
    MachineKinematics machine;
    MachineAxis c, a;
    c.name = "C";
    a.name = "A";
    a.vector[0] = 1;
    a.vector[2] = 0;
    a.sequence = 1;
    a.min = -120;
    a.max = 30;
    machine.SetAxes({}, {c, a});
    GCodeParser parser;
    parser.SetMachine(&machine);
    parser.AddLine("G0 A0 C0 X5 Y6 Z7");
    parser.AddLine("G0 A-90 X5 Y-30 Z20");
    const auto& m = parser.Operations;
    ASSERT_EQ(m.size(), 2u);
    double axis[3];
    MotionToolAxis(m[0], axis);
    EXPECT_NEAR(axis[2], 1, 1e-12);
    EXPECT_NEAR(m[0].y, 6, 1e-12);
    MotionToolAxis(m[1], axis);
    EXPECT_NEAR(axis[1], -1, 1e-9);
    // A -90 puts a point (x, y, z) of the part at (x, z, -y): the point (5, -30, 20) of the
    // machine is (5, -20, -30) in the part
    EXPECT_NEAR(m[1].x, 5, 1e-9);
    EXPECT_NEAR(m[1].y, -20, 1e-9);
    EXPECT_NEAR(m[1].z, -30, 1e-9);
    // the commands of a FreeCAD toolpath keep their rotary words as MotionToolAxis has them
    parser.AddCommand("G0", {{"A", 90}, {"X", 0}});
    MotionToolAxis(m.back(), axis);
    EXPECT_NEAR(axis[1], -1, 1e-9);
    EXPECT_NEAR(m.back().y, -30, 1e-12);
}

// NOLINTEND(cppcoreguidelines-*,readability-*)
