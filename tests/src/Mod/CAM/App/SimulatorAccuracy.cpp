// SPDX-License-Identifier: LGPL-2.1-or-later

#include "SimulatorAccuracy.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <thread>

#include <Mod/CAM/PathSimulator/AppGL/MaterialMesher.h>
#include <Mod/CAM/PathSimulator/AppGL/MaterialSimulation.h>
#include <Mod/CAM/PathSimulator/AppGL/MaterialTool.h>
#include <Mod/CAM/PathSimulator/AppGL/ThreadPool.h>

namespace CAMSimulator::Accuracy
{

namespace
{

using V3 = Point;

V3 Sub(const V3& a, const V3& b)
{
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
V3 Add(const V3& a, const V3& b)
{
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
V3 Mul(const V3& a, double s)
{
    return {a[0] * s, a[1] * s, a[2] * s};
}
double Dot(const V3& a, const V3& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
V3 Cross(const V3& a, const V3& b)
{
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double Norm(const V3& a)
{
    return std::sqrt(Dot(a, a));
}
V3 Unit(const V3& a)
{
    const double l = Norm(a);
    return l > 0 ? Mul(a, 1 / l) : V3 {0, 0, 1};
}
// v rotated about the unit axis k by an angle
V3 Rotate(const V3& v, const V3& k, double angle)
{
    const double c = std::cos(angle), s = std::sin(angle);
    return Add(Add(Mul(v, c), Mul(Cross(k, v), s)), Mul(k, Dot(k, v) * (1 - c)));
}

double Ms(std::chrono::steady_clock::time_point since)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - since).count();
}

int Threads(int requested)
{
    return requested > 0 ? requested : std::max(1, (int)std::thread::hardware_concurrency());
}

void ParallelRun(int threads, const std::function<void(int)>& work)
{
    std::vector<std::thread> pool;
    for (int k = 0; k < threads; k++) {
        pool.emplace_back(work, k);
    }
    for (auto& t : pool) {
        t.join();
    }
}

// A tool: the solid of revolution of a closed polygon in (radius, height)
struct Tool
{
    std::vector<std::pair<double, double>> polygon;
    double radius = 0, zmin = 0, zmax = 0;
    // the edges of the polygon (from vertex i to i + 1) overlapping each slice of the height
    static constexpr int Slices = 512;
    std::vector<std::vector<uint32_t>> slices;

    explicit Tool(const std::vector<float>& profile)
    {
        for (size_t i = 0; i + 1 < profile.size(); i += 2) {
            polygon.emplace_back(std::max(0.0f, profile[i]), profile[i + 1]);
        }
        if (polygon.empty()) {
            return;
        }
        if (polygon.front().first > 0) {
            polygon.insert(polygon.begin(), {0.0, polygon.front().second});
        }
        if (polygon.back().first > 0) {
            polygon.emplace_back(0.0, polygon.back().second);
        }
        zmin = 1e30;
        zmax = -1e30;
        for (const auto& p : polygon) {
            radius = std::max(radius, p.first);
            zmin = std::min(zmin, p.second);
            zmax = std::max(zmax, p.second);
        }
        slices.resize(Slices);
        for (size_t i = 0; i < polygon.size(); i++) {
            const auto& a = polygon[i];
            const auto& b = polygon[(i + 1) % polygon.size()];
            const int s0 = Slice(std::min(a.second, b.second)),
                      s1 = Slice(std::max(a.second, b.second));
            for (int k = s0; k <= s1; k++) {
                slices[k].push_back((uint32_t)i);
            }
        }
    }
    int Slice(double z) const
    {
        const double h = std::max(zmax - zmin, 1e-9);
        return std::clamp((int)std::floor((z - zmin) / h * Slices), 0, Slices - 1);
    }
    // a probe: all of its solid below the tip, it doesn't cut
    bool Cuts() const
    {
        return !polygon.empty() && zmax > 1e-3;
    }
    bool Inside(double r, double z) const
    {
        if (polygon.empty() || z < zmin || z > zmax) {
            return false;
        }
        bool in = false;
        for (uint32_t i : slices[Slice(z)]) {
            const auto& a = polygon[i];
            const auto& b = polygon[(i + 1) % polygon.size()];
            if ((a.second > z) != (b.second > z)) {
                const double x = a.first
                    + (z - a.second) * (b.first - a.first) / (b.second - a.second);
                if (r < x) {
                    in = !in;
                }
            }
        }
        return in;
    }
    // Does the segment q0 + s (q1 - q0), s in [0, 1], in the frame of the tool (axis along z)
    // enter the solid: at its ends, or between the parameters where it crosses the surfaces of
    // revolution of the edges
    bool SegmentHits(const V3& q0, const V3& q1) const
    {
        auto at = [&](double s, double& r, double& z) {
            const V3 q = Add(q0, Mul(Sub(q1, q0), s));
            z = q[2];
            r = std::hypot(q[0], q[1]);
        };
        double r, z;
        at(0, r, z);
        if (Inside(r, z)) {
            return true;
        }
        at(1, r, z);
        if (Inside(r, z)) {
            return true;
        }
        const V3 d = Sub(q1, q0);
        if (std::max(q0[2], q1[2]) < zmin || std::min(q0[2], q1[2]) > zmax) {
            return false;
        }
        // squared distance from the axis: A s^2 + B s + C
        const double A = d[0] * d[0] + d[1] * d[1];
        const double B = 2 * (q0[0] * d[0] + q0[1] * d[1]);
        const double C = q0[0] * q0[0] + q0[1] * q0[1];
        if (A > 0) {
            const double s = std::clamp(-B / (2 * A), 0.0, 1.0);
            if (A * s * s + B * s + C > radius * radius) {
                return false;
            }
        }
        else if (C > radius * radius) {
            return false;
        }
        thread_local std::vector<double> params;
        params.assign({0.0, 1.0});
        const size_t n = polygon.size();
        // the edges in the height range of the segment (an edge in several slices adds the same
        // parameters again, which doesn't matter)
        const int s0 = Slice(std::min(q0[2], q1[2])), s1 = Slice(std::max(q0[2], q1[2]));
        for (int slice = s0; slice <= s1; slice++) {
            for (uint32_t i : slices[slice]) {
                const auto& a = polygon[i];
                const auto& b = polygon[(i + 1) % n];
                if (a.first == 0 && b.first == 0) {
                    continue;
                }
                const double dz = b.second - a.second;
                if (std::abs(dz) < 1e-12) {
                    if (std::abs(d[2]) > 1e-15) {
                        const double s = (a.second - q0[2]) / d[2];
                        if (s > 0 && s < 1) {
                            params.push_back(s);
                        }
                    }
                    continue;
                }
                // the cone of the edge: radius r0 + r1 s along the segment
                const double k = (b.first - a.first) / dz;
                const double r0 = a.first + (q0[2] - a.second) * k;
                const double r1 = d[2] * k;
                const double qa = r1 * r1 - A, qb = 2 * r0 * r1 - B, qc = r0 * r0 - C;
                if (std::abs(qa) < 1e-15) {
                    if (std::abs(qb) > 1e-15) {
                        const double s = -qc / qb;
                        if (s > 0 && s < 1) {
                            params.push_back(s);
                        }
                    }
                    continue;
                }
                double disc = qb * qb - 4 * qa * qc;
                if (disc < 0) {
                    continue;
                }
                disc = std::sqrt(disc);
                for (double s : {(-qb - disc) / (2 * qa), (-qb + disc) / (2 * qa)}) {
                    if (s > 0 && s < 1) {
                        params.push_back(s);
                    }
                }
            }
        }
        std::sort(params.begin(), params.end());
        for (size_t i = 0; i + 1 < params.size(); i++) {
            if (params[i + 1] - params[i] < 1e-14) {
                continue;
            }
            for (double s : {0.5 * (params[i] + params[i + 1]), params[i + 1]}) {
                at(s, r, z);
                if (Inside(r, z)) {
                    return true;
                }
            }
        }
        return false;
    }
};

// The profile of a tool with its arcs as the spheres of the material simulation, finely sampled
std::vector<float> FineProfile(const Material::Tool& tool)
{
    std::vector<double> zs;
    for (const auto& f : tool.Frustums()) {
        zs.push_back(f.z0);
        zs.push_back(f.z1);
    }
    for (const auto& s : tool.Spheres()) {
        for (int i = 0; i <= 2000; i++) {
            zs.push_back(s.zc - s.radius + 2 * s.radius * i / 2000);
        }
    }
    std::sort(zs.begin(), zs.end());
    std::vector<float> profile;
    for (int i = (int)zs.size() - 1; i >= 0; i--) {
        const double z = zs[i];
        if (z < tool.MinZ() - 1e-9 || z > tool.MaxZ() + 1e-9) {
            continue;
        }
        const double above = std::max(0.0, tool.RadiusAt(z + 1e-9));
        const double below = std::max(0.0, tool.RadiusAt(z - 1e-9));
        profile.push_back((float)above);
        profile.push_back((float)z);
        if (std::abs(above - below) > 1e-7) {
            profile.push_back((float)below);
            profile.push_back((float)z);
        }
    }
    return profile;
}

// The profile of a tool with its arcs, as the job gives them, finely sampled: the exact tool
std::vector<float> ArcProfile(const std::vector<float>& profile, const std::vector<Material::ProfileArc>& arcs)
{
    const int n = (int)profile.size() / 2;
    std::vector<Material::ProfileArc> sorted(arcs.begin(), arcs.end());
    std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });
    std::vector<float> out;
    int next = 0;
    auto point = [&](int i) {
        return std::make_pair((double)profile[2 * i], (double)profile[2 * i + 1]);
    };
    for (const auto& a : sorted) {
        if (a.first < next || a.last <= a.first || a.last >= n) {
            continue;
        }
        for (; next <= a.first; next++) {
            out.insert(out.end(), {profile[2 * next], profile[2 * next + 1]});
        }
        const auto [r0, z0] = point(a.first);
        const auto [r1, z1] = point(a.last);
        const auto [rm, zm] = point((a.first + a.last) / 2);
        const double radius = std::hypot(r0 - a.r, z0 - a.z);
        const double t0 = std::atan2(z0 - a.z, r0 - a.r);
        double sweep = std::atan2(z1 - a.z, r1 - a.r) - t0;
        double mid = std::atan2(zm - a.z, rm - a.r) - t0;
        // both in (-pi, pi]: the arc goes the way of the point in its middle
        sweep = std::atan2(std::sin(sweep), std::cos(sweep));
        mid = std::atan2(std::sin(mid), std::cos(mid));
        if (a.last - a.first >= 2 && (mid * sweep < 0 || std::abs(mid) > std::abs(sweep))) {
            sweep += sweep > 0 ? -2 * M_PI : 2 * M_PI;
        }
        for (int k = 1; k < 2000; k++) {
            const double t = t0 + sweep * k / 2000;
            out.push_back((float)std::max(0.0, a.r + radius * std::cos(t)));
            out.push_back((float)(a.z + radius * std::sin(t)));
        }
        next = a.last;
    }
    for (; next < n; next++) {
        out.insert(out.end(), {profile[2 * next], profile[2 * next + 1]});
    }
    return out;
}

// A piece of a motion: the tool moves straight and turns about a fixed axis
struct Piece
{
    int tool;
    V3 p0, p1;     // tip positions
    V3 a0;         // tool axis at the start
    V3 turnAxis;   // rotation taking the axis at the start to the end
    double angle;  // of the rotation, 0 for a fixed axis
};

// Point in the triangle closest to p, its distance (Ericson, Real-Time Collision Detection)
double PointTriangleDistance(const V3& p, const V3& a, const V3& b, const V3& c)
{
    const V3 ab = Sub(b, a), ac = Sub(c, a), ap = Sub(p, a);
    const double d1 = Dot(ab, ap), d2 = Dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) {
        return Norm(ap);
    }
    const V3 bp = Sub(p, b);
    const double d3 = Dot(ab, bp), d4 = Dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) {
        return Norm(bp);
    }
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        return Norm(Sub(p, Add(a, Mul(ab, d1 / (d1 - d3)))));
    }
    const V3 cp = Sub(p, c);
    const double d5 = Dot(ab, cp), d6 = Dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) {
        return Norm(cp);
    }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        return Norm(Sub(p, Add(a, Mul(ac, d2 / (d2 - d6)))));
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return Norm(Sub(p, Add(b, Mul(Sub(c, b), w))));
    }
    const double denom = 1 / (va + vb + vc);
    return Norm(Sub(p, Add(a, Add(Mul(ab, vb * denom), Mul(ac, vc * denom)))));
}

V3 Vertex(const Mesh& mesh, uint32_t i)
{
    return {mesh.xyz[3 * i], mesh.xyz[3 * i + 1], mesh.xyz[3 * i + 2]};
}

// Distance from the mesh, with the triangles in a uniform grid
class MeshDistance
{
public:
    MeshDistance(const Mesh& mesh, const V3& lo, const V3& hi, double cell)
        : mMesh(mesh)
        , mCell(cell)
        , mLo(lo)
    {
        for (int k = 0; k < 3; k++) {
            mN[k] = (int)std::ceil((hi[k] - lo[k]) / cell) + 1;
        }
        mCells.resize((size_t)mN[0] * mN[1] * mN[2]);
        for (size_t t = 0; t < mesh.indices.size() / 3; t++) {
            int i0[3], i1[3];
            Range(t, i0, i1);
            for (int z = i0[2]; z <= i1[2]; z++) {
                for (int y = i0[1]; y <= i1[1]; y++) {
                    for (int x = i0[0]; x <= i1[0]; x++) {
                        mCells[Index(x, y, z)].push_back((uint32_t)t);
                    }
                }
            }
        }
    }
    // up to maxDistance
    double Distance(const V3& p, double maxDistance) const
    {
        const int r = (int)std::ceil(maxDistance / mCell);
        int c[3];
        for (int k = 0; k < 3; k++) {
            c[k] = (int)std::floor((p[k] - mLo[k]) / mCell);
        }
        double best = maxDistance;
        for (int z = std::max(0, c[2] - r); z <= std::min(mN[2] - 1, c[2] + r); z++) {
            for (int y = std::max(0, c[1] - r); y <= std::min(mN[1] - 1, c[1] + r); y++) {
                for (int x = std::max(0, c[0] - r); x <= std::min(mN[0] - 1, c[0] + r); x++) {
                    for (uint32_t t : mCells[Index(x, y, z)]) {
                        const uint32_t* v = &mMesh.indices[3 * t];
                        best = std::min(
                            best,
                            PointTriangleDistance(
                                p,
                                Vertex(mMesh, v[0]),
                                Vertex(mMesh, v[1]),
                                Vertex(mMesh, v[2])
                            )
                        );
                    }
                }
            }
        }
        return best;
    }

private:
    size_t Index(int x, int y, int z) const
    {
        return ((size_t)z * mN[1] + y) * mN[0] + x;
    }
    void Range(size_t t, int i0[3], int i1[3]) const
    {
        for (int k = 0; k < 3; k++) {
            double lo = 1e30, hi = -1e30;
            for (int c = 0; c < 3; c++) {
                const double v = mMesh.xyz[3 * mMesh.indices[3 * t + c] + k];
                lo = std::min(lo, v);
                hi = std::max(hi, v);
            }
            i0[k] = std::clamp((int)std::floor((lo - mLo[k]) / mCell), 0, mN[k] - 1);
            i1[k] = std::clamp((int)std::floor((hi - mLo[k]) / mCell), 0, mN[k] - 1);
        }
    }

    const Mesh& mMesh;
    double mCell;
    V3 mLo;
    int mN[3];
    std::vector<std::vector<uint32_t>> mCells;
};

// Inside the mesh by the parity of its crossings along +Z, with the triangles in columns
class MeshInside
{
public:
    MeshInside(const Mesh& mesh, const V3& lo, const V3& hi, double cell)
        : mMesh(mesh)
        , mCell(cell)
        , mLo(lo)
    {
        mN[0] = (int)std::ceil((hi[0] - lo[0]) / cell) + 1;
        mN[1] = (int)std::ceil((hi[1] - lo[1]) / cell) + 1;
        mCells.resize((size_t)mN[0] * mN[1]);
        for (size_t t = 0; t < mesh.indices.size() / 3; t++) {
            int i0[2], i1[2];
            for (int k = 0; k < 2; k++) {
                double a = 1e30, b = -1e30;
                for (int c = 0; c < 3; c++) {
                    const double v = mesh.xyz[3 * mesh.indices[3 * t + c] + k];
                    a = std::min(a, v);
                    b = std::max(b, v);
                }
                i0[k] = std::clamp((int)std::floor((a - lo[k]) / cell), 0, mN[k] - 1);
                i1[k] = std::clamp((int)std::floor((b - lo[k]) / cell), 0, mN[k] - 1);
            }
            for (int y = i0[1]; y <= i1[1]; y++) {
                for (int x = i0[0]; x <= i1[0]; x++) {
                    mCells[(size_t)y * mN[0] + x].push_back((uint32_t)t);
                }
            }
        }
    }
    bool Inside(const V3& p) const
    {
        const int x = std::clamp((int)std::floor((p[0] - mLo[0]) / mCell), 0, mN[0] - 1);
        const int y = std::clamp((int)std::floor((p[1] - mLo[1]) / mCell), 0, mN[1] - 1);
        int crossings = 0;
        for (uint32_t t : mCells[(size_t)y * mN[0] + x]) {
            const float* a = &mMesh.xyz[3 * mMesh.indices[3 * t]];
            const float* b = &mMesh.xyz[3 * mMesh.indices[3 * t + 1]];
            const float* c = &mMesh.xyz[3 * mMesh.indices[3 * t + 2]];
            auto edge = [&](const float* u, const float* v) {
                return (v[0] - u[0]) * (p[1] - u[1]) - (v[1] - u[1]) * (p[0] - u[0]);
            };
            const double e0 = edge(a, b), e1 = edge(b, c), e2 = edge(c, a);
            if (!(e0 > 0 && e1 > 0 && e2 > 0) && !(e0 < 0 && e1 < 0 && e2 < 0)) {
                continue;
            }
            const double z = (e1 * a[2] + e2 * b[2] + e0 * c[2]) / (e0 + e1 + e2);
            if (z > p[2]) {
                crossings++;
            }
        }
        return (crossings & 1) != 0;
    }

private:
    const Mesh& mMesh;
    double mCell;
    V3 mLo;
    int mN[2];
    std::vector<std::vector<uint32_t>> mCells;
};

}  // namespace

struct Oracle::Data
{
    // deviation of the pieces from the motions in the frame of the tool
    static constexpr double Tolerance = 1e-4;
    // cells of the grid of the pieces
    static constexpr double Cell = 2;

    std::vector<Tool> tools;
    std::vector<Piece> pieces;
    std::vector<V3> stockVertices;
    std::vector<std::array<uint32_t, 3>> stockTriangles;
    V3 lo, hi;
    int n[3] = {1, 1, 1};
    // pieces by the cells their box overlaps, with a margin of one cell around the stock
    std::vector<std::vector<uint32_t>> grid;

    void AddSegment(const Material::Segment& seg, double tEnd)
    {
        const Tool& tool = tools[seg.tool];
        if (!tool.Cuts()) {
            return;
        }
        const V3 a0 = Unit({seg.axis0[0], seg.axis0[1], seg.axis0[2]});
        const V3 a1 = Unit({seg.axis1[0], seg.axis1[1], seg.axis1[2]});
        // a chord of an arc of angle da and radius r deviates from it by r da^2 / 8: turning the
        // axis moves the points of the tool on arcs up to its reach, an arc of the path moves the
        // tip on its radius
        const double reach
            = std::hypot(std::max(std::abs(tool.zmax), std::abs(tool.zmin)), tool.radius) + 1;
        const double turn = std::acos(std::clamp(Dot(a0, a1), -1.0, 1.0)) * tEnd;
        const double arcTurn = seg.arc ? std::abs(seg.sweep) * tEnd : 0;
        const double n1 = turn > 0 ? turn / std::sqrt(8 * Tolerance / reach) : 0;
        const double n2 = arcTurn > 0
            ? arcTurn / std::sqrt(8 * Tolerance / std::max(1e-6, (double)seg.radius))
            : 0;
        const int count = std::max(1, (int)std::ceil(n1 + n2));
        auto position = [&](double t) -> V3 {
            const double z = seg.start[2] + t * ((double)seg.end[2] - seg.start[2]);
            if (seg.arc) {
                const double angle = seg.startAngle + t * seg.sweep;
                return {
                    seg.center[0] - std::sin(angle) * seg.radius,
                    seg.center[1] + std::cos(angle) * seg.radius,
                    z
                };
            }
            return {
                seg.start[0] + t * ((double)seg.end[0] - seg.start[0]),
                seg.start[1] + t * ((double)seg.end[1] - seg.start[1]),
                z
            };
        };
        auto axis = [&](double t) {
            return Unit(Add(Mul(a0, 1 - t), Mul(a1, t)));
        };
        for (int i = 0; i < count; i++) {
            const double t0 = tEnd * i / count, t1 = tEnd * (i + 1) / count;
            Piece pc;
            pc.tool = seg.tool;
            pc.p0 = position(t0);
            pc.p1 = position(t1);
            const V3 b0 = axis(t0), b1 = axis(t1);
            pc.a0 = b0;
            const V3 c = Cross(b0, b1);
            const double s = Norm(c);
            pc.angle = std::atan2(s, Dot(b0, b1));
            pc.turnAxis = s > 1e-12 ? Mul(c, 1 / s) : V3 {0, 0, 0};
            pieces.push_back(pc);
        }
    }

    void PieceBox(const Piece& pc, V3& blo, V3& bhi) const
    {
        const Tool& t = tools[pc.tool];
        blo = {1e30, 1e30, 1e30};
        bhi = {-1e30, -1e30, -1e30};
        const V3 a1 = pc.angle > 0 ? Rotate(pc.a0, pc.turnAxis, pc.angle) : pc.a0;
        for (int e = 0; e < 2; e++) {
            const V3& p = e ? pc.p1 : pc.p0;
            const V3& a = e ? a1 : pc.a0;
            for (double z : {t.zmin, t.zmax}) {
                const V3 c = Add(p, Mul(a, z));
                for (int k = 0; k < 3; k++) {
                    const double r = t.radius * std::sqrt(std::max(0.0, 1 - a[k] * a[k])) + 1e-3;
                    blo[k] = std::min(blo[k], c[k] - r);
                    bhi[k] = std::max(bhi[k], c[k] + r);
                }
            }
        }
        // the bulge of a turning tool beyond the boxes at the ends
        const double reach = std::hypot(std::max(std::abs(t.zmax), std::abs(t.zmin)), t.radius);
        const double bulge = reach * pc.angle * pc.angle / 8 + 1e-3;
        for (int k = 0; k < 3; k++) {
            blo[k] -= bulge;
            bhi[k] += bulge;
        }
    }

    size_t Index(int x, int y, int z) const
    {
        return ((size_t)z * n[1] + y) * n[0] + x;
    }

    void BuildGrid()
    {
        for (int k = 0; k < 3; k++) {
            n[k] = std::max(1, (int)std::ceil((hi[k] - lo[k]) / Cell) + 2);
        }
        grid.assign((size_t)n[0] * n[1] * n[2], {});
        for (size_t i = 0; i < pieces.size(); i++) {
            V3 blo, bhi;
            PieceBox(pieces[i], blo, bhi);
            int i0[3], i1[3];
            bool outside = false;
            for (int k = 0; k < 3; k++) {
                i0[k] = std::max(0, (int)std::floor((blo[k] - lo[k]) / Cell) + 1);
                i1[k] = std::min(n[k] - 1, (int)std::floor((bhi[k] - lo[k]) / Cell) + 1);
                outside = outside || i0[k] > i1[k];
            }
            if (outside) {
                continue;
            }
            for (int z = i0[2]; z <= i1[2]; z++) {
                for (int y = i0[1]; y <= i1[1]; y++) {
                    for (int x = i0[0]; x <= i1[0]; x++) {
                        grid[Index(x, y, z)].push_back((uint32_t)i);
                    }
                }
            }
        }
    }

    bool InStock(const V3& p) const
    {
        for (int k = 0; k < 3; k++) {
            if (p[k] < lo[k] || p[k] > hi[k]) {
                return false;
            }
        }
        // parity of the crossings of a ray in a direction no face of a usual stock is parallel to
        const V3 d = Unit({0.3141, 0.2718, 1.0});
        int crossings = 0;
        for (const auto& t : stockTriangles) {
            const V3& v0 = stockVertices[t[0]];
            const V3 e1 = Sub(stockVertices[t[1]], v0), e2 = Sub(stockVertices[t[2]], v0);
            const V3 h = Cross(d, e2);
            const double a = Dot(e1, h);
            if (std::abs(a) < 1e-15) {
                continue;
            }
            const double f = 1 / a;
            const V3 s = Sub(p, v0);
            const double u = f * Dot(s, h);
            if (u < 0 || u > 1) {
                continue;
            }
            const V3 q = Cross(s, e1);
            const double v = f * Dot(d, q);
            if (v < 0 || u + v > 1) {
                continue;
            }
            if (f * Dot(e2, q) > 0) {
                crossings++;
            }
        }
        return (crossings & 1) != 0;
    }

    bool InPiece(const Piece& pc, const V3& p) const
    {
        // the point in the frame of the tool at the start of the piece, and at its end turned back
        // by the rotation of the tool since the start
        const V3 v0 = Sub(p, pc.p0);
        V3 v1 = Sub(p, pc.p1);
        if (pc.angle > 0) {
            v1 = Rotate(v1, pc.turnAxis, -pc.angle);
        }
        const V3& ez = pc.a0;
        const V3 ex = Unit(std::abs(ez[0]) < 0.9 ? Cross(ez, {1, 0, 0}) : Cross(ez, {0, 1, 0}));
        const V3 ey = Cross(ez, ex);
        const V3 q0 = {Dot(v0, ex), Dot(v0, ey), Dot(v0, ez)};
        const V3 q1 = {Dot(v1, ex), Dot(v1, ey), Dot(v1, ez)};
        return tools[pc.tool].SegmentHits(q0, q1);
    }
};

Oracle::Oracle(const Material::Job& job, long upToStep, bool polylineTools)
    : d(new Data)
{
    for (size_t t = 0; t < job.profiles.size(); t++) {
        const auto& profile = job.profiles[t];
        // the arcs of the job, else the arcs the material simulation fits
        if (!polylineTools && t < job.arcs.size() && !job.arcs[t].empty()) {
            d->tools.emplace_back(ArcProfile(profile, job.arcs[t]));
            continue;
        }
        const Material::Tool fitted(profile);
        const bool fine = !polylineTools && fitted.Cuts() && !fitted.Spheres().empty();
        d->tools.emplace_back(fine ? FineProfile(fitted) : profile);
    }
    for (size_t i = 0; i + 2 < job.stockXyz.size(); i += 3) {
        d->stockVertices.push_back({job.stockXyz[i], job.stockXyz[i + 1], job.stockXyz[i + 2]});
    }
    for (size_t i = 0; i + 2 < job.stockTriangles.size(); i += 3) {
        d->stockTriangles.push_back(
            {job.stockTriangles[i], job.stockTriangles[i + 1], job.stockTriangles[i + 2]}
        );
    }
    d->lo = {1e30, 1e30, 1e30};
    d->hi = {-1e30, -1e30, -1e30};
    for (const V3& v : d->stockVertices) {
        for (int k = 0; k < 3; k++) {
            d->lo[k] = std::min(d->lo[k], v[k]);
            d->hi[k] = std::max(d->hi[k], v[k]);
        }
    }
    long step = 0;
    for (const Material::Segment& seg : job.segments) {
        double tEnd = 1;
        if (upToStep >= 0) {
            if (step >= upToStep) {
                break;
            }
            if (step + seg.steps > upToStep) {
                tEnd = seg.StepParam((int)(upToStep - step));
            }
        }
        step += seg.steps;
        d->AddSegment(seg, tEnd);
    }
    d->BuildGrid();
}

Oracle::~Oracle()
{
    delete d;
}

bool Oracle::Material(const Point& p) const
{
    if (!d->InStock(p)) {
        return false;
    }
    int c[3];
    for (int k = 0; k < 3; k++) {
        c[k] = (int)std::floor((p[k] - d->lo[k]) / Data::Cell) + 1;
        if (c[k] < 0 || c[k] >= d->n[k]) {
            return true;
        }
    }
    for (uint32_t i : d->grid[d->Index(c[0], c[1], c[2])]) {
        if (d->InPiece(d->pieces[i], p)) {
            return false;
        }
    }
    return true;
}

const Point& Oracle::Lo() const
{
    return d->lo;
}

const Point& Oracle::Hi() const
{
    return d->hi;
}

size_t Oracle::PieceCount() const
{
    return d->pieces.size();
}

Report Evaluate(const Oracle& oracle, const Mesh& mesh, const Options& options)
{
    Report report;
    const int threads = Threads(options.threads);
    const size_t triangles = mesh.indices.size() / 3;

    // surface error: samples uniform in area
    std::vector<double> area(triangles + 1, 0);
    for (size_t t = 0; t < triangles; t++) {
        const uint32_t* v = &mesh.indices[3 * t];
        const V3 a = Vertex(mesh, v[0]), b = Vertex(mesh, v[1]), c = Vertex(mesh, v[2]);
        area[t + 1] = area[t] + 0.5 * Norm(Cross(Sub(b, a), Sub(c, a)));
    }
    const int samples = triangles > 0 && area.back() > 0 ? options.surfaceSamples : 0;
    constexpr double Missed = 1e30;
    std::vector<double> errors(samples, 0);
    std::vector<V3> where(samples);
    std::atomic<int> next {0};
    const double step = std::min(0.005, options.searchRange / 10);
    ParallelRun(threads, [&](int) {
        std::mt19937_64 rng;
        std::uniform_real_distribution<double> uniform(0, 1);
        for (int i; (i = next++) < samples;) {
            rng.seed(1000003ull * i + 7);
            const double at = uniform(rng) * area.back();
            size_t t = std::upper_bound(area.begin(), area.end(), at) - area.begin() - 1;
            t = std::min(t, triangles - 1);
            double u = uniform(rng), v = uniform(rng);
            if (u + v > 1) {
                u = 1 - u;
                v = 1 - v;
            }
            const uint32_t* idx = &mesh.indices[3 * t];
            const V3 a = Vertex(mesh, idx[0]), b = Vertex(mesh, idx[1]), c = Vertex(mesh, idx[2]);
            const V3 p = Add(a, Add(Mul(Sub(b, a), u), Mul(Sub(c, a), v)));
            const V3 n = Unit(Cross(Sub(b, a), Sub(c, a)));
            where[i] = p;
            // in the material the true surface is farther out, else farther in
            const bool in = oracle.Material(p);
            const double dir = in ? 1 : -1;
            // distance along a direction to where the material changes, negative if not in range
            auto changeAlong = [&](const V3& d) {
                double lo = 0, hi = -1;
                for (double s = step; s <= options.searchRange + 1e-9; s += step) {
                    if (oracle.Material(Add(p, Mul(d, s))) != in) {
                        lo = s - step;
                        hi = s;
                        break;
                    }
                }
                if (hi < 0) {
                    return -1.0;
                }
                for (int it = 0; it < 22; it++) {
                    const double mid = 0.5 * (lo + hi);
                    if (oracle.Material(Add(p, Mul(d, mid))) == in) {
                        lo = mid;
                    }
                    else {
                        hi = mid;
                    }
                }
                return 0.5 * (lo + hi);
            };
            // along the normal, outwards in the material: the true surface is farther out
            double distance = changeAlong(Mul(n, dir));
            if (distance < 0) {
                // A sample on a surface along another one, like a wall at the height of the floor
                // it meets, searches along that surface: the nearest change along the axes and
                // their diagonals
                for (int k = 0; k < 27; k++) {
                    if (k == 13) {
                        continue;
                    }
                    const V3 d = Unit(
                        {(double)(k % 3 - 1), (double)(k / 3 % 3 - 1), (double)(k / 9 - 1)}
                    );
                    const double along = changeAlong(d);
                    if (along >= 0 && (distance < 0 || along < distance)) {
                        distance = along;
                    }
                }
            }
            if (distance < 0) {
                errors[i] = Missed;
                continue;
            }
            errors[i] = dir * distance;
        }
    });
    std::vector<double> magnitude;
    double sum = 0, largest = -1;
    for (int i = 0; i < samples; i++) {
        const double e = errors[i];
        if (e == Missed) {
            report.missed++;
            report.missedAt.push_back(where[i]);
            continue;
        }
        if (std::abs(e) > largest) {
            largest = std::abs(e);
            report.maxAt = where[i];
        }
        magnitude.push_back(std::abs(e));
        sum += e;
    }
    std::sort(magnitude.begin(), magnitude.end());
    report.surfaceSamples = (int)magnitude.size();
    if (!magnitude.empty()) {
        auto percentile = [&](double q) {
            return magnitude[std::min(magnitude.size() - 1, (size_t)(q * magnitude.size()))];
        };
        double total = 0;
        for (double m : magnitude) {
            total += m;
        }
        report.mean = total / magnitude.size();
        report.p50 = percentile(0.5);
        report.p90 = percentile(0.9);
        report.p99 = percentile(0.99);
        report.p999 = percentile(0.999);
        report.max = magnitude.back();
        report.bias = sum / magnitude.size();
    }

    // volume error
    const V3 lo = oracle.Lo(), hi = oracle.Hi();
    const V3 margin = {1, 1, 1};
    const MeshDistance distance(mesh, Sub(lo, margin), Add(hi, margin), 0.5);
    const MeshInside inside(mesh, Sub(lo, margin), Add(hi, margin), 0.25);
    std::atomic<long> wrong {0}, gross {0}, material {0};
    std::mutex mutex;
    std::atomic<long> nextPoint {0};
    const long points = options.volumePoints;
    ParallelRun(threads, [&](int) {
        std::mt19937_64 rng;
        std::uniform_real_distribution<double> uniform(0, 1);
        for (long i; (i = nextPoint++) < points;) {
            rng.seed(7919ull * i + 13);
            V3 p;
            for (int k = 0; k < 3; k++) {
                p[k] = lo[k] + uniform(rng) * (hi[k] - lo[k]);
            }
            const bool truth = oracle.Material(p);
            if (truth) {
                material++;
            }
            if (truth == inside.Inside(p)) {
                continue;
            }
            wrong++;
            if (distance.Distance(p, options.grossDistance) >= options.grossDistance) {
                gross++;
                std::lock_guard<std::mutex> lock(mutex);
                report.grossPoints.push_back(p);
                report.grossMaterial.push_back(truth);
            }
        }
    });
    report.volumePoints = points;
    report.boxVolume = (hi[0] - lo[0]) * (hi[1] - lo[1]) * (hi[2] - lo[2]);
    if (points > 0) {
        const double perPoint = report.boxVolume / points;
        report.materialVolume = perPoint * material;
        report.misclassifiedVolume = perPoint * wrong;
        report.gross = gross;
        report.grossVolume = perPoint * gross;
    }
    return report;
}

std::string Format(const Report& r)
{
    char buffer[512];
    std::string out;
    std::snprintf(
        buffer,
        sizeof(buffer),
        "surface: samples %d  mean %.4f  p50 %.4f  p90 %.4f  p99 %.4f  p999 %.4f  max %.4f  bias "
        "%+.4f  "
        "missed %d (mm), max at %.3f %.3f %.3f\n",
        r.surfaceSamples,
        r.mean,
        r.p50,
        r.p90,
        r.p99,
        r.p999,
        r.max,
        r.bias,
        r.missed,
        r.maxAt[0],
        r.maxAt[1],
        r.maxAt[2]
    );
    out += buffer;
    std::snprintf(
        buffer,
        sizeof(buffer),
        "volume: points %ld  material %.1f mm3  misclassified %.2f mm3 (%.4f%%)  gross %ld (%.2f "
        "mm3)\n",
        r.volumePoints,
        r.materialVolume,
        r.misclassifiedVolume,
        r.boxVolume > 0 ? 100 * r.misclassifiedVolume / r.boxVolume : 0.0,
        r.gross,
        r.grossVolume
    );
    out += buffer;
    return out;
}

Mesh SimulateMaterial(const Material::Job& job, float quality, long step, RunStats* stats)
{
    using clock = std::chrono::steady_clock;
    std::vector<std::shared_ptr<const Material::Tool>> tools;
    for (size_t t = 0; t < job.profiles.size(); t++) {
        static const std::vector<Material::ProfileArc> none;
        const auto& arcs = t < job.arcs.size() ? job.arcs[t] : none;
        tools.push_back(std::make_shared<Material::Tool>(job.profiles[t], true, arcs));
    }
    Material::Simulation sim;
    sim.SetFluteHeights(job.flutes);
    const auto start = clock::now();
    sim.Start(tools, job.segments, job.stockXyz, job.stockTriangles, quality);
    const long target = step < 0 ? sim.TotalSteps() : std::min(step, sim.TotalSteps());
    sim.SetTarget(target);
    Material::Grid grid;
    std::vector<int> changed;
    for (;;) {
        while (sim.Pending()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (sim.GetLayout()) {
            if (!grid.LayoutRef()) {
                grid.Init(sim.GetLayout());
            }
            if (sim.TakeDisplay(grid, changed) == target) {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const double simulateMs = Ms(start);

    const auto meshStart = clock::now();
    ThreadPool pool;
    Material::Mesher mesher;
    mesher.Reset(sim.GetLayout(), pool);
    mesher.Update(grid, changed, pool);
    Mesh out;
    for (int u = 0; u < mesher.UnitCount(); u++) {
        const Material::UnitMesh& unit = mesher.Mesh(u);
        const auto base = (uint32_t)(out.xyz.size() / 3);
        for (const auto& v : unit.vertices) {
            out.xyz.insert(out.xyz.end(), {v.x, v.y, v.z});
        }
        for (const auto* list : {&unit.stock, &unit.cut}) {
            for (uint32_t i : *list) {
                out.indices.push_back(base + i);
            }
        }
    }
    if (stats) {
        stats->simulateMs = simulateMs;
        stats->meshMs = Ms(meshStart);
        stats->triangles = (long)(out.indices.size() / 3);
    }
    sim.Stop();
    return out;
}

}  // namespace CAMSimulator::Accuracy
