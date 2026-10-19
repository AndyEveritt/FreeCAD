// SPDX-License-Identifier: LGPL-2.1-or-later

// The toolpath of the CAM simulator: commands and G-code text into motions

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <Mod/CAM/PathSimulator/AppGL/GCodeParser.h>

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

TEST(SimulatorGCode, arcsOutsideTheXYPlaneAreLines)
{
    // G18: a half circle in the XZ plane around (0, 0, 0), from X10 to X-10, over the top: seen
    // from +Y, counterclockwise from +Z towards +X
    const auto m = Lines({"G0 X10 Y3 Z0", "G18 G2 X-10 Z0 I-10 K0"});
    ASSERT_GT(m.size(), 10u);
    for (size_t i = 1; i < m.size(); i++) {
        EXPECT_EQ(m[i].cmd, eMoveLiner);
        EXPECT_NEAR(std::hypot(m[i].x, m[i].z), 10, 1e-9);
        EXPECT_DOUBLE_EQ(m[i].y, 3);
        // clockwise in the ZX plane from +X goes through +Z
        EXPECT_GE(m[i].z, -1e-9);
        // the chords are within the tolerance of the arc
        const double mx = (m[i].x + m[i - 1].x) / 2, mz = (m[i].z + m[i - 1].z) / 2;
        EXPECT_LE(10 - std::hypot(mx, mz), GCodeParser::ArcTolerance + 1e-9);
    }
    EXPECT_DOUBLE_EQ(m.back().x, -10);
    EXPECT_DOUBLE_EQ(m.back().z, 0);
    // G19 in the YZ plane, counterclockwise seen from +X: from +Y towards +Z
    const auto n = Lines({"G0 X0 Y5 Z0", "G19 G3 Y-5 Z0 J-5 K0"});
    ASSERT_GT(n.size(), 3u);
    EXPECT_GT(n[n.size() / 2].z, 4);
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

// NOLINTEND(cppcoreguidelines-*,readability-*)
