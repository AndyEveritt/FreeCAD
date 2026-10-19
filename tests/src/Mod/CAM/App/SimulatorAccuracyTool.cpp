// SPDX-License-Identifier: LGPL-2.1-or-later

// CAM_simulator_accuracy: the accuracy of the CAM simulator on a job written by the GUI
// (environment variable CAM_SIM_DUMP), see SimulatorAccuracy.h and the README of the benchmark.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <Mod/CAM/PathSimulator/AppGL/MaterialJob.h>

#include "SimulatorAccuracy.h"

using namespace CAMSimulator;

namespace
{

const char* Usage = R"(usage: CAM_simulator_accuracy JOB [options]

Simulates the job outside of the GUI and measures the stock shown against the exact material.

  --quality Q               simulator accuracy 1..10 (default: the quality of the job)
  --step N                  timeline step to measure (default: the end)
  --samples N               surface samples (default 20000)
  --points N                volume points (default 1000000)
  --polyline-tools          tools as their profiles, arcs not fitted
  --json FILE               write the results as JSON
  --gross FILE              write the points of gross errors (x y z material) and of samples
                            without the true surface nearby (x y z missed)
  --max-p99 MM              fail above this 99th percentile of the surface error
  --max-p999 MM             fail above this 99.9th percentile of the surface error
  --max-gross MM3           fail above this volume of gross errors
  --max-missed N            fail above this number of samples without the true surface nearby

Exit status: 0 within the limits, 1 beyond them, 2 for errors.
)";

}  // namespace

int main(int argc, char** argv)
{
    std::string jobPath, jsonPath, grossPath;
    double quality = -1, maxP99 = -1, maxP999 = -1, maxGross = -1;
    long step = -1, maxMissed = -1;
    bool polyline = false;
    Accuracy::Options options;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        auto value = [&]() -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s needs a value\n", a.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "-h" || a == "--help") {
            std::fputs(Usage, stdout);
            return 0;
        }
        else if (a == "--quality") {
            quality = std::atof(value());
        }
        else if (a == "--step") {
            step = std::atol(value());
        }
        else if (a == "--samples") {
            options.surfaceSamples = std::atoi(value());
        }
        else if (a == "--points") {
            options.volumePoints = std::atol(value());
        }
        else if (a == "--polyline-tools") {
            polyline = true;
        }
        else if (a == "--json") {
            jsonPath = value();
        }
        else if (a == "--gross") {
            grossPath = value();
        }
        else if (a == "--max-p99") {
            maxP99 = std::atof(value());
        }
        else if (a == "--max-p999") {
            maxP999 = std::atof(value());
        }
        else if (a == "--max-gross") {
            maxGross = std::atof(value());
        }
        else if (a == "--max-missed") {
            maxMissed = std::atol(value());
        }
        else if (!a.empty() && a[0] != '-' && jobPath.empty()) {
            jobPath = a;
        }
        else {
            std::fprintf(stderr, "unknown argument %s\n%s", a.c_str(), Usage);
            return 2;
        }
    }
    if (jobPath.empty()) {
        std::fputs(Usage, stderr);
        return 2;
    }
    Material::Job job;
    if (!Material::LoadJob(jobPath, job)) {
        std::fprintf(stderr, "can't read the job %s\n", jobPath.c_str());
        return 2;
    }
    const float q = quality > 0 ? (float)quality : job.quality;
    long steps = 0;
    for (const auto& s : job.segments) {
        steps += s.steps;
    }
    std::printf(
        "job %s: %zu tools, %zu segments, %ld steps, quality %.0f\n",
        jobPath.c_str(),
        job.profiles.size(),
        job.segments.size(),
        steps,
        q
    );
    std::fflush(stdout);

    Accuracy::RunStats stats;
    const Accuracy::Mesh mesh = Accuracy::SimulateMaterial(job, q, step, &stats);
    std::printf(
        "simulation %.0f ms, mesh %.0f ms, %ld triangles\n",
        stats.simulateMs,
        stats.meshMs,
        stats.triangles
    );
    std::fflush(stdout);
    const Accuracy::Oracle oracle(job, step, polyline);
    const Accuracy::Report r = Accuracy::Evaluate(oracle, mesh, options);
    std::fputs(Accuracy::Format(r).c_str(), stdout);

    if (!grossPath.empty()) {
        if (FILE* f = std::fopen(grossPath.c_str(), "w")) {
            for (size_t i = 0; i < r.grossPoints.size(); i++) {
                const auto& p = r.grossPoints[i];
                std::fprintf(f, "%.4f %.4f %.4f %d\n", p[0], p[1], p[2], r.grossMaterial[i] ? 1 : 0);
            }
            for (const auto& p : r.missedAt) {
                std::fprintf(f, "%.4f %.4f %.4f missed\n", p[0], p[1], p[2]);
            }
            std::fclose(f);
        }
    }
    if (!jsonPath.empty()) {
        FILE* f = std::fopen(jsonPath.c_str(), "w");
        if (!f) {
            std::fprintf(stderr, "can't write %s\n", jsonPath.c_str());
            return 2;
        }
        std::fprintf(
            f,
            "{\"quality\": %g, \"step\": %ld, \"simulate_ms\": %.1f, "
            "\"mesh_ms\": %.1f, "
            "\"triangles\": %ld, \"surface\": {\"samples\": %d, \"mean\": %.6f, \"p50\": %.6f, "
            "\"p90\": %.6f, "
            "\"p99\": %.6f, \"p999\": %.6f, \"max\": %.6f, \"bias\": %.6f, \"missed\": %d}, "
            "\"volume\": {\"points\": %ld, \"material_mm3\": %.3f, \"misclassified_mm3\": %.3f, "
            "\"gross\": %ld, "
            "\"gross_mm3\": %.3f}}\n",
            q,
            step,
            stats.simulateMs,
            stats.meshMs,
            stats.triangles,
            r.surfaceSamples,
            r.mean,
            r.p50,
            r.p90,
            r.p99,
            r.p999,
            r.max,
            r.bias,
            r.missed,
            r.volumePoints,
            r.materialVolume,
            r.misclassifiedVolume,
            r.gross,
            r.grossVolume
        );
        std::fclose(f);
    }

    bool ok = true;
    auto limit = [&](const char* name, double value, double max) {
        if (max >= 0 && value > max) {
            std::printf("FAIL: %s %.4f above %.4f\n", name, value, max);
            ok = false;
        }
    };
    limit("p99", r.p99, maxP99);
    limit("p999", r.p999, maxP999);
    limit("gross mm3", r.grossVolume, maxGross);
    limit("missed", (double)r.missed, (double)maxMissed);
    return ok ? 0 : 1;
}
