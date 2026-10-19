// SPDX-License-Identifier: LGPL-2.1-or-later

// Accuracy of the material simulation against the exact material of small jobs (the oracle of
// SimulatorAccuracy.h), and of the oracle itself against a cut known in closed form

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <Mod/CAM/PathSimulator/AppGL/MaterialJob.h>

#include "MaterialTestTools.h"
#include "SimulatorAccuracy.h"

// NOLINTBEGIN(cppcoreguidelines-*,readability-*)

using namespace CAMSimulator;
using namespace MaterialTest;

namespace
{

Material::Segment Line(int tool, float x0, float y0, float z0, float x1, float y1, float z1)
{
    Material::Segment s;
    s.tool = tool;
    s.start[0] = x0;
    s.start[1] = y0;
    s.start[2] = z0;
    s.end[0] = x1;
    s.end[1] = y1;
    s.end[2] = z1;
    return s;
}

// a counterclockwise full circle around (cx, cy) at height z, starting at angle 0
Material::Segment Circle(int tool, float cx, float cy, float r, float z)
{
    Material::Segment s;
    s.tool = tool;
    s.arc = true;
    s.center[0] = cx;
    s.center[1] = cy;
    s.radius = r;
    s.startAngle = 0;
    s.sweep = 2 * (float)M_PI;
    s.start[0] = s.end[0] = cx;
    s.start[1] = s.end[1] = cy + r;
    s.start[2] = s.end[2] = z;
    return s;
}

void SetAxes(Material::Segment& s, const double a0[3], const double a1[3])
{
    for (int k = 0; k < 3; k++) {
        s.axis0[k] = (float)a0[k];
        s.axis1[k] = (float)a1[k];
    }
}

Material::Job BoxJob(const double lo[3], const double hi[3])
{
    Material::Job job;
    MakeBox(lo, hi, job.stockXyz, job.stockTriangles);
    return job;
}

// 3 axis: a pocket with a flat end mill along lines and arcs, a 3D surface finished with a ball
// end (as FreeCAD gives it: its arc in 8 edges), a slot of a bull nose and an engraving of a V-bit
Material::Job ThreeAxisJob()
{
    const double lo[3] = {0, 0, -20}, hi[3] = {60, 40, 0};
    Material::Job job = BoxJob(lo, hi);
    job.profiles = {FlatProfile(3), BallProfile(3, 8), BullNoseProfile(4, 1), VBitProfile(4, 45)};
    auto& s = job.segments;
    for (float y = 6; y <= 16; y += 4) {
        s.push_back(Line(0, 6, y, -4, 24, y, -4));
    }
    s.push_back(Circle(0, 15, 11, 7, -6));
    s.push_back(Circle(0, 15, 11, 3, -8));
    // ridges between passes of the ball end, on a slope and a wave
    for (float x = 32; x <= 56; x += 1.5f) {
        const float z = -3 - 0.15f * (x - 32);
        for (int i = 0; i < 12; i++) {
            const float y0 = 4 + i * 1.5f, y1 = y0 + 1.5f;
            s.push_back(Line(1, x, y0, z + std::sin(y0 / 3), x, y1, z + std::sin(y1 / 3)));
        }
    }
    s.push_back(Line(2, 4, 26, -2.5f, 28, 34, -5));
    s.push_back(Line(3, 32, 30, -1, 56, 34, -2));
    s.push_back(Line(3, 56, 34, -2, 40, 37, -0.5f));
    return job;
}

// 5 axis: a ball end and a flat end mill turning while they move, and a flat end mill on a tilted
// work plane (3+2)
Material::Job FiveAxisJob()
{
    const double lo[3] = {0, 0, -20}, hi[3] = {50, 40, 0};
    Material::Job job = BoxJob(lo, hi);
    job.profiles = {BallProfile(3, 8), FlatProfile(2.5f)};
    auto& s = job.segments;
    const double up[3] = {0, 0, 1};
    for (int i = 0; i < 5; i++) {
        const double tilt = 0.6 * i / 4;
        const double a1[3] = {std::sin(tilt), 0, std::cos(tilt)};
        Material::Segment m = Line(0, 5, 5 + 4 * i, -3, 25, 7 + 4 * i, -4);
        SetAxes(m, up, a1);
        s.push_back(m);
    }
    const double tiltX[3] = {0, -std::sin(0.5), std::cos(0.5)};
    const double tiltY[3] = {std::sin(0.4), 0, std::cos(0.4)};
    Material::Segment turn = Line(1, 30, 6, -2, 45, 18, -3);
    SetAxes(turn, tiltX, tiltY);
    s.push_back(turn);
    const double plane[3] = {0, std::sin(0.35), std::cos(0.35)};
    for (float x = 8; x <= 40; x += 4) {
        Material::Segment m = Line(1, x, 28, -1.5f, x + 3, 36, -1.5f);
        SetAxes(m, plane, plane);
        s.push_back(m);
    }
    return job;
}

Accuracy::Report Measure(const Material::Job& job, float quality)
{
    const Accuracy::Mesh mesh = Accuracy::SimulateMaterial(job, quality);
    const Accuracy::Oracle oracle(job);
    Accuracy::Options options;
    options.surfaceSamples = 5000;
    options.volumePoints = 200000;
    const Accuracy::Report r = Accuracy::Evaluate(oracle, mesh, options);
    std::printf("%s", Accuracy::Format(r).c_str());
    return r;
}

}  // namespace

// The oracle against cuts known in closed form: a slot of a flat end mill, and a ball end moving
// down diagonally, whose sphere sweeps a capsule and whose shank (above the center of the sphere)
// the vertical walls along the path of the center
TEST(MaterialAccuracy, oracleIsExact)
{
    const double lo[3] = {0, 0, -10}, hi[3] = {40, 20, 0};
    Material::Job job = BoxJob(lo, hi);
    const double r = 2;
    job.profiles = {FlatProfile(3), BallProfile((float)r, 4)};
    job.segments = {Line(0, 5, 10, -4, 25, 10, -4), Line(1, 28, 4, -2, 36, 16, -6)};
    const Accuracy::Oracle oracle(job);
    const double c0[3] = {28, 4, -2 + r}, c1[3] = {36, 16, -6 + r};

    // signed distance in XY from the segment of the center positions for t in [t0, t1]
    auto xyDistance = [&](const Accuracy::Point& p, double t0, double t1) {
        const double dx = c1[0] - c0[0], dy = c1[1] - c0[1];
        double t = ((p[0] - c0[0]) * dx + (p[1] - c0[1]) * dy) / (dx * dx + dy * dy);
        t = std::clamp(t, t0, t1);
        return std::hypot(p[0] - c0[0] - t * dx, p[1] - c0[1] - t * dy);
    };
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> u(0, 1);
    int checked = 0, wrong = 0;
    for (int i = 0; i < 200000; i++) {
        const Accuracy::Point p = {lo[0] + u(rng) * 40, lo[1] + u(rng) * 20, lo[2] + u(rng) * 10};
        // slot: within 3 of the segment in XY, above its bottom
        const double sx = std::clamp(p[0], 5.0, 25.0);
        const double slot = std::min(3 - std::hypot(p[0] - sx, p[1] - 10), p[2] + 4);
        // capsule: within r of the segment of the centers
        double best = 1e30;
        {
            const double d[3] = {c1[0] - c0[0], c1[1] - c0[1], c1[2] - c0[2]};
            double t = ((p[0] - c0[0]) * d[0] + (p[1] - c0[1]) * d[1] + (p[2] - c0[2]) * d[2])
                / (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            t = std::clamp(t, 0.0, 1.0);
            best = std::hypot(p[0] - c0[0] - t * d[0], p[1] - c0[1] - t * d[1], p[2] - c0[2] - t * d[2]);
        }
        const double capsule = r - best;
        // shank: above the center at some time of the motion, within r of it in XY; the center
        // descends, so it is above it from the time the center is at its height on
        double shank = -1e30, shankZ = 1e30;
        const double tz = std::clamp((c0[2] - p[2]) / (c0[2] - c1[2]), 0.0, 1.0);
        if (p[2] >= c1[2]) {
            shank = r - xyDistance(p, tz, 1);
            shankZ = std::abs(p[2] - c0[2]) < std::abs(p[2] - c1[2]) ? p[2] - c0[2] : 1e30;
        }
        // points within 1 um of a boundary are not checked
        constexpr double Margin = 1e-3;
        if (std::abs(slot) < Margin || std::abs(capsule) < Margin || std::abs(shank) < Margin
            || std::abs(shankZ) < Margin) {
            continue;
        }
        const bool removed = slot > 0 || capsule > 0 || shank > 0;
        checked++;
        wrong += oracle.Material(p) == removed ? 1 : 0;
    }
    EXPECT_GT(checked, 190000);
    EXPECT_EQ(wrong, 0);
}

TEST(MaterialAccuracy, threeAxis)
{
    const Accuracy::Report r = Measure(ThreeAxisJob(), 10);
    EXPECT_LT(r.p99, 0.02);
    EXPECT_LT(r.p999, 0.06);
    EXPECT_LT(r.grossVolume, 0.05);
    EXPECT_EQ(r.missed, 0);
}

TEST(MaterialAccuracy, fiveAxis)
{
    const Accuracy::Report r = Measure(FiveAxisJob(), 10);
    EXPECT_LT(r.p99, 0.02);
    EXPECT_LT(r.p999, 0.06);
    EXPECT_LT(r.grossVolume, 0.05);
    EXPECT_EQ(r.missed, 0);
}

// NOLINTEND(cppcoreguidelines-*,readability-*)
