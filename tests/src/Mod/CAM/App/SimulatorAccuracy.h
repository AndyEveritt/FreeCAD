// SPDX-License-Identifier: LGPL-2.1-or-later

// Accuracy validation of the CAM simulator: the stock a simulation shows against the exact
// material of its job.
//
// The oracle decides for any point whether it holds material after the toolpath: inside the stock
// mesh and outside every tool swept along the path. It shares no code with the material simulation:
// tools are solids of revolution of their profile, every motion is divided into pieces whose paths
// in the frame of the tool deviate from straight lines by less than 0.1 um, and a point is in the
// sweep of a piece if the segment between its positions in the tool frame at both ends meets the
// tool. Only the arcs of the profiles (ball ends, corner radii) are taken as the spheres the
// material simulation fits to them (Material::Tool), sampled finely, as profiles approximate arcs
// by a few edges.
//
// The evaluation measures the mesh of the stock against it:
// - surface error: points sampled uniformly on the mesh triangles, the distance along the mesh
//   normal to the true surface (bracketed in steps, then bisected), positive where the mesh is
//   inside the material (shows too much removed);
// - volume error: random points in the stock box classified by the oracle and by the mesh (ray
//   parity along Z); misclassified points farther than a threshold from the mesh are gross errors,
//   like a missing or wrongly placed piece of surface.

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <Mod/CAM/PathSimulator/AppGL/MaterialJob.h>

namespace CAMSimulator::Accuracy
{

using Point = std::array<double, 3>;

// A triangle mesh, counterclockwise seen from outside
struct Mesh
{
    std::vector<float> xyz;
    std::vector<uint32_t> indices;
};

class Oracle
{
public:
    // The material after the given timeline step of the job, after the whole path if negative.
    // With polylineTools the profiles are taken as given, without the fitted arcs.
    explicit Oracle(const Material::Job& job, long upToStep = -1, bool polylineTools = false);
    ~Oracle();
    Oracle(const Oracle&) = delete;
    Oracle& operator=(const Oracle&) = delete;

    bool Material(const Point& p) const;
    // bounding box of the stock
    const Point& Lo() const;
    const Point& Hi() const;
    size_t PieceCount() const;

    struct Data;

private:
    Data* d;
};

struct Options
{
    int surfaceSamples = 20000;
    long volumePoints = 1000000;
    // the true surface is searched up to this distance from the mesh
    double searchRange = 0.5;
    // misclassified points farther from the mesh are gross errors
    double grossDistance = 0.05;
    int threads = 0;  // 0: all cores
};

struct Report
{
    // surface error (mm) of the samples with the true surface in the search range
    int surfaceSamples = 0;
    double mean = 0, p50 = 0, p90 = 0, p99 = 0, p999 = 0, max = 0;
    double bias = 0;  // mean signed error, positive: too much material removed
    int missed = 0;   // samples without the true surface in the search range
    // volume error
    long volumePoints = 0;
    double boxVolume = 0;  // mm3
    double materialVolume = 0;
    double misclassifiedVolume = 0;
    long gross = 0;
    double grossVolume = 0;
    std::vector<Point> grossPoints;
    std::vector<bool> grossMaterial;  // the oracle has material at the gross point
};

Report Evaluate(const Oracle& oracle, const Mesh& mesh, const Options& options = {});

// Summary lines of a report
std::string Format(const Report& report);

// The stock shown by the simulation, simulated outside the GUI
struct RunStats
{
    double simulateMs = 0;  // start to the requested step shown
    double meshMs = 0;
    long triangles = 0;
};

// The material simulation at a step of the timeline (the end if negative)
Mesh SimulateMaterial(const Material::Job& job, float quality, long step = -1, RunStats* stats = nullptr);

}  // namespace CAMSimulator::Accuracy
