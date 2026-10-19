// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2026 The FreeCAD project association AISBL              *
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

#include "MaterialTool.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace CAMSimulator::Material
{

namespace
{

constexpr double Inf = std::numeric_limits<double>::infinity();
// angle between the edges of a profile up to which it is smooth there (about 25 degrees)
constexpr double SmoothCos = 0.9;

inline double Dot(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void Normalize(double v[3])
{
    const double len = std::sqrt(Dot(v, v));
    if (len > 0) {
        v[0] /= len;
        v[1] /= len;
        v[2] /= len;
    }
    else {
        v[0] = v[1] = 0;
        v[2] = 1;
    }
}

// Orthonormal frame around a unit axis
void Frame(const double a[3], double e1[3], double e2[3])
{
    // the world axis least aligned with a
    double ref[3] = {0, 0, 0};
    const double ax = std::abs(a[0]), ay = std::abs(a[1]), az = std::abs(a[2]);
    ref[ax <= ay && ax <= az ? 0 : (ay <= az ? 1 : 2)] = 1;
    e1[0] = ref[1] * a[2] - ref[2] * a[1];
    e1[1] = ref[2] * a[0] - ref[0] * a[2];
    e1[2] = ref[0] * a[1] - ref[1] * a[0];
    Normalize(e1);
    e2[0] = a[1] * e1[2] - a[2] * e1[1];
    e2[1] = a[2] * e1[0] - a[0] * e1[2];
    e2[2] = a[0] * e1[1] - a[1] * e1[0];
}

// Real roots of a x^2 + 2 b x + c = 0, returns their number
int SolveQuadratic(double a, double b, double c, double roots[2])
{
    const double scale = std::max({std::abs(a), std::abs(b), std::abs(c)});
    if (scale == 0) {
        return 0;
    }
    if (std::abs(a) <= 1e-14 * scale) {
        if (std::abs(b) <= 1e-14 * scale) {
            return 0;
        }
        roots[0] = -c / (2 * b);
        return 1;
    }
    double disc = b * b - a * c;
    if (disc < 0) {
        if (disc < -1e-12 * std::max(b * b, std::abs(a * c))) {
            return 0;
        }
        disc = 0;
    }
    const double q = -(b + std::copysign(std::sqrt(disc), b));
    if (q == 0) {
        roots[0] = 0;
        return 1;
    }
    roots[0] = q / a;
    roots[1] = c / q;
    return 2;
}

}  // namespace

// ---------------------------------------------------------------------------------------- Tool

Tool::Tool(const std::vector<float>& profile, bool cuts, const std::vector<ProfileArc>& arcs)
    : mCuts(cuts)
{
    std::vector<std::pair<double, double>> given;
    for (size_t i = 0; i + 1 < profile.size(); i += 2) {
        given.emplace_back(std::max(0.0, (double)profile[i]), (double)profile[i + 1]);
    }
    // the arcs in order along the profile, each from the end of the one before
    std::vector<ProfileArc> sorted;
    for (const ProfileArc& a : arcs) {
        if (a.first >= 0 && a.last > a.first && a.last < (int)given.size()) {
            sorted.push_back(a);
        }
    }
    std::sort(sorted.begin(), sorted.end(), [](const ProfileArc& a, const ProfileArc& b) {
        return a.first < b.first;
    });
    std::vector<std::pair<double, double>> points;
    int next = 0;  // the next given point
    for (const ProfileArc& a : sorted) {
        if (a.first < next) {
            continue;  // overlaps the arc before
        }
        for (; next <= a.first; next++) {
            points.push_back(given[next]);
        }
        const auto [r0, z0] = given[a.first];
        const auto [r1, z1] = given[a.last];
        const double radius = std::hypot(r0 - a.r, z0 - a.z);
        const double a0 = std::atan2(z0 - a.z, r0 - a.r);
        double sweep = std::remainder(std::atan2(z1 - a.z, r1 - a.r) - a0, 2 * M_PI);
        // the arc through the given points between the ends, the shorter one without them
        if (a.last - a.first >= 2) {
            const auto [rm, zm] = given[(a.first + a.last) / 2];
            const double am = std::remainder(std::atan2(zm - a.z, rm - a.r) - a0, 2 * M_PI);
            if (am * sweep < 0 || std::abs(am) > std::abs(sweep)) {
                sweep += sweep > 0 ? -2 * M_PI : 2 * M_PI;
            }
        }
        const double step = std::max(
            radius > ArcTolerance ? 2 * std::acos(1 - ArcTolerance / radius) : M_PI / 2,
            M_PI / 64
        );
        const int n = std::max(3, (int)std::ceil(std::abs(sweep) / step));
        for (int k = 1; k < n; k++) {
            const double angle = a0 + sweep * k / n;
            points.emplace_back(
                std::max(0.0, a.r + radius * std::cos(angle)),
                a.z + radius * std::sin(angle)
            );
        }
        next = a.last;
    }
    for (; next < (int)given.size(); next++) {
        points.push_back(given[next]);
    }
    Build(std::move(points));
}

void Tool::Build(std::vector<std::pair<double, double>> points)
{
    if (points.empty()) {
        mCuts = false;
        return;
    }
    // closed polygon in the (radius, height) half plane
    if (points.front().first > 0) {
        points.insert(points.begin(), {0.0, points.front().second});
    }
    if (points.back().first > 0) {
        points.emplace_back(0.0, points.back().second);
    }
    mPolygon = points;
    mMinZ = Inf;
    mMaxZ = -Inf;
    mRadius = 0;
    for (const auto& p : points) {
        mMinZ = std::min(mMinZ, p.second);
        mMaxZ = std::max(mMaxZ, p.second);
        mRadius = std::max(mRadius, p.first);
    }
    // probes are modeled below their tip
    if (mMaxZ <= 1e-6 || mRadius <= 0 || mMaxZ <= mMinZ) {
        mCuts = false;
    }

    // The solid as a stack of frustums: between neighboring vertex heights, the largest radius
    // of the polygon is linear (the hull of the cross sections, exact for every profile with one
    // radius per height).
    std::vector<double> heights;
    for (const auto& p : points) {
        heights.push_back(p.second);
    }
    std::sort(heights.begin(), heights.end());
    heights.erase(
        std::unique(heights.begin(), heights.end(), [](double a, double b) { return b - a < 1e-9; }),
        heights.end()
    );
    const size_t n = points.size();
    // outer radius in a slab, at its bottom and its top
    auto slabRadius = [&](double za, double zb, double& ra, double& rb) {
        ra = rb = -1;
        const double zm = 0.5 * (za + zb);
        double best = -1;
        for (size_t i = 0; i < n; i++) {
            const auto& p = points[i];
            const auto& q = points[(i + 1) % n];
            const double lo = std::min(p.second, q.second), hi = std::max(p.second, q.second);
            if (lo > za + 1e-9 || hi < zb - 1e-9 || hi - lo < 1e-12) {
                continue;
            }
            auto at = [&](double z) {
                return p.first + (z - p.second) * (q.first - p.first) / (q.second - p.second);
            };
            const double rm = at(zm);
            if (rm > best) {
                best = rm;
                ra = std::max(0.0, at(za));
                rb = std::max(0.0, at(zb));
            }
        }
        return best > 0;
    };
    mFrustums.clear();
    for (size_t i = 0; i + 1 < heights.size(); i++) {
        Frustum f {};
        f.z0 = heights[i];
        f.z1 = heights[i + 1];
        if (!slabRadius(f.z0, f.z1, f.r0, f.r1)) {
            continue;
        }
        mFrustums.push_back(f);
    }

    // normals: outward edge normals, averaged at smooth joints
    auto edgeNormal = [](const Frustum& f, double& nr, double& nz) {
        const double dz = f.z1 - f.z0, dr = f.r1 - f.r0;
        const double len = std::hypot(dz, dr);
        nr = dz / len;
        nz = -dr / len;
    };
    const size_t nf = mFrustums.size();
    for (size_t i = 0; i < nf; i++) {
        Frustum& f = mFrustums[i];
        double nr, nz;
        edgeNormal(f, nr, nz);
        f.n0r = f.n1r = nr;
        f.n0z = f.n1z = nz;
        f.sharp0 = f.sharp1 = true;
    }
    for (size_t i = 0; i + 1 < nf; i++) {
        Frustum& a = mFrustums[i];
        Frustum& b = mFrustums[i + 1];
        if (std::abs(a.z1 - b.z0) > 1e-9 || std::abs(a.r1 - b.r0) > 1e-6 * std::max(1.0, a.r1)) {
            continue;  // a step: the rims are sharp
        }
        double anr, anz, bnr, bnz;
        edgeNormal(a, anr, anz);
        edgeNormal(b, bnr, bnz);
        if (anr * bnr + anz * bnz < SmoothCos) {
            continue;
        }
        double mr = anr + bnr, mz = anz + bnz;
        const double len = std::hypot(mr, mz);
        mr /= len;
        mz /= len;
        a.n1r = b.n0r = mr;
        a.n1z = b.n0z = mz;
        a.sharp1 = b.sharp0 = false;
    }
    // the tip on the axis of a flat or round end is smooth, of a cone sharp
    for (Frustum& f : mFrustums) {
        if (f.r0 <= 1e-9 && f.sharp0 && std::abs(f.n0z) > 0.99) {
            f.sharp0 = false;
            f.n0r = 0;
            f.n0z = -1;
        }
        if (f.r1 <= 1e-9 && f.sharp1 && std::abs(f.n1z) > 0.99) {
            f.sharp1 = false;
            f.n1r = 0;
            f.n1z = 1;
        }
    }

    // Arcs of the profile centered on the axis are spheres: runs of at least 3 smoothly joined
    // frustums with all vertices on a circle around a point of the axis. Their sweeps are exact
    // and cheaper than those of the faceted arc. A sphere replaces its frustums if the parts of
    // the sphere beyond the run are inside the rest of the tool, like the upper half of a ball
    // end inside the shank.
    mSpheres.clear();
    auto outerRadius = [&](double z) {
        double r = -1;
        for (const Frustum& f : mFrustums) {
            if (z >= f.z0 - 1e-9 && z <= f.z1 + 1e-9) {
                const double u = f.z1 > f.z0 ? (z - f.z0) / (f.z1 - f.z0) : 0;
                r = std::max(r, f.r0 + std::clamp(u, 0.0, 1.0) * (f.r1 - f.r0));
            }
        }
        return r;
    };
    std::vector<bool> replaced(nf, false);
    for (size_t i = 0; i + 2 < nf;) {
        // center from the first three vertices of the run
        auto vertex = [&](size_t k) {
            // vertex k of the run: bottom of frustum i + k, or the top of the last
            return k == 0 ? std::make_pair(mFrustums[i].r0, mFrustums[i].z0)
                          : std::make_pair(mFrustums[i + k - 1].r1, mFrustums[i + k - 1].z1);
        };
        auto [ra, za] = vertex(0);
        auto [rb, zb] = vertex(2);
        size_t end = i;  // frustums [i, end) form the run
        double zc = 0, radius = 0;
        if (std::abs(za - zb) > 1e-9) {
            zc = (rb * rb - ra * ra + zb * zb - za * za) / (2 * (zb - za));
            radius = std::hypot(ra, za - zc);
            const double tol = 1e-4 * radius + 1e-6;
            end = i;
            while (end < nf) {
                const Frustum& f = mFrustums[end];
                if (end > i && mFrustums[end - 1].sharp1) {
                    break;
                }
                if (std::abs(std::hypot(f.r0, f.z0 - zc) - radius) > tol
                    || std::abs(std::hypot(f.r1, f.z1 - zc) - radius) > tol) {
                    break;
                }
                end++;
            }
        }
        if (end - i < 3 || radius <= 0) {
            i++;
            continue;
        }
        // the sphere beyond the run must be inside the rest of the tool
        const double zLo = mFrustums[i].z0, zHi = mFrustums[end - 1].z1;
        bool contained = true;
        for (int k = 0; k <= 64 && contained; k++) {
            const double z = zc - radius + 2 * radius * k / 64;
            if (z >= zLo - 1e-9 && z <= zHi + 1e-9) {
                continue;
            }
            const double r = std::sqrt(std::max(0.0, radius * radius - (z - zc) * (z - zc)));
            contained = outerRadius(z) >= r - 1e-4 * radius;
        }
        if (!contained) {
            i++;
            continue;
        }
        // the sphere fitted to all vertices of the run: r^2 + z^2 = 2 z zc + (R^2 - zc^2)
        {
            double sz = 0, sy = 0, szz = 0, szy = 0;
            int count = 0;
            auto add = [&](double r, double z) {
                const double y = r * r + z * z;
                sz += z;
                sy += y;
                szz += z * z;
                szy += z * y;
                count++;
            };
            add(mFrustums[i].r0, mFrustums[i].z0);
            for (size_t k = i; k < end; k++) {
                add(mFrustums[k].r1, mFrustums[k].z1);
            }
            const double det = count * szz - sz * sz;
            if (det > 1e-12) {
                const double slope = (count * szy - sz * sy) / det;
                const double intercept = (sy - slope * sz) / count;
                zc = slope / 2;
                radius = std::sqrt(std::max(0.0, intercept + zc * zc));
            }
        }
        mSpheres.push_back({zc, radius});
        for (size_t k = i; k < end; k++) {
            replaced[k] = true;
        }
        i = end;
    }
    if (!mSpheres.empty()) {
        std::vector<Frustum> kept;
        for (size_t i = 0; i < nf; i++) {
            if (!replaced[i]) {
                kept.push_back(mFrustums[i]);
            }
        }
        mFrustums.swap(kept);
    }

    // lines along the axis: the intervals of the primitives at a distance, merged, at distances
    // between and at the radii of the primitives
    std::vector<double> radii = {0};
    for (const Frustum& f : mFrustums) {
        radii.push_back(f.r0);
        radii.push_back(f.r1);
    }
    for (const Sphere& s : mSpheres) {
        radii.push_back(s.radius);
    }
    std::sort(radii.begin(), radii.end());
    std::vector<double> samples;
    for (size_t i = 0; i < radii.size(); i++) {
        samples.push_back(radii[i]);
        if (i + 1 < radii.size()) {
            for (int k = 1; k < 4; k++) {
                samples.push_back(radii[i] + (radii[i + 1] - radii[i]) * k / 4);
            }
        }
    }
    mSingleAxial = true;
    for (double rho : samples) {
        std::vector<std::pair<double, double>> iv;
        for (const Frustum& f : mFrustums) {
            const double rmax = std::max(f.r0, f.r1), rmin = std::min(f.r0, f.r1);
            if (rho > rmax + 1e-12) {
                continue;
            }
            double lo = f.z0, hi = f.z1;
            if (rho > rmin && rmax > rmin) {
                const double z = f.z0 + (rho - f.r0) * (f.z1 - f.z0) / (f.r1 - f.r0);
                if (f.r1 > f.r0) {
                    lo = z;
                }
                else {
                    hi = z;
                }
            }
            iv.emplace_back(lo, hi);
        }
        for (const Sphere& s : mSpheres) {
            if (rho <= s.radius) {
                const double h = std::sqrt(s.radius * s.radius - rho * rho);
                iv.emplace_back(s.zc - h, s.zc + h);
            }
        }
        std::sort(iv.begin(), iv.end());
        for (size_t i = 1; i < iv.size(); i++) {
            // gaps from the joints of fitted spheres are not undercuts
            if (iv[i].first > iv[i - 1].second + 1e-4 * mRadius) {
                mSingleAxial = false;
            }
            iv[i].second = std::max(iv[i].second, iv[i - 1].second);
        }
    }
}

double Tool::RadiusAt(double z) const
{
    double r = -1;
    for (const Frustum& f : mFrustums) {
        if (z >= f.z0 && z <= f.z1) {
            const double u = f.z1 > f.z0 ? (z - f.z0) / (f.z1 - f.z0) : 0;
            r = std::max(r, f.r0 + u * (f.r1 - f.r0));
        }
    }
    for (const Sphere& s : mSpheres) {
        const double d = z - s.zc;
        if (std::abs(d) <= s.radius) {
            r = std::max(r, std::sqrt(s.radius * s.radius - d * d));
        }
    }
    return r;
}

double Tool::BottomAt(double rho) const
{
    double z = Inf;
    for (const Frustum& f : mFrustums) {
        const double rmax = std::max(f.r0, f.r1), rmin = std::min(f.r0, f.r1);
        if (rho > rmax) {
            continue;
        }
        double lo = f.z0;
        if (f.r1 > f.r0 && rho > rmin) {
            lo = f.z0 + (rho - f.r0) * (f.z1 - f.z0) / (f.r1 - f.r0);
        }
        z = std::min(z, lo);
    }
    for (const Sphere& s : mSpheres) {
        if (rho <= s.radius) {
            z = std::min(z, s.zc - std::sqrt(s.radius * s.radius - rho * rho));
        }
    }
    return z;
}

double Tool::MaxBottom(double rho) const
{
    // the bottom is piecewise monotone between the radii of the primitives
    double z = BottomAt(0);
    std::vector<double> radii;
    for (const Frustum& f : mFrustums) {
        radii.push_back(f.r0);
        radii.push_back(f.r1);
    }
    for (const Sphere& s : mSpheres) {
        radii.push_back(s.radius);
    }
    for (double r : radii) {
        if (r <= rho) {
            z = std::max(z, BottomAt(r));
        }
    }
    return std::max(z, BottomAt(rho));
}

double Tool::RadiusBelow(double height) const
{
    const double top = mMinZ + height;
    double r = 0;
    for (const Frustum& f : mFrustums) {
        if (f.z0 <= top) {
            r = std::max(r, f.r0);
            const double u = f.z1 > f.z0 ? std::clamp((top - f.z0) / (f.z1 - f.z0), 0.0, 1.0) : 1;
            r = std::max(r, f.r0 + u * (f.r1 - f.r0));
        }
    }
    for (const Sphere& s : mSpheres) {
        const double z = std::clamp(s.zc, s.zc - s.radius, top);
        const double d = z - s.zc;
        if (std::abs(d) <= s.radius) {
            r = std::max(r, std::sqrt(s.radius * s.radius - d * d));
        }
    }
    return std::min(r, mRadius);
}

double Tool::FeatureSize() const
{
    const double height = std::min(mRadius, mMaxZ - mMinZ);
    const double r = RadiusAt(mMinZ + height / 2);
    return std::clamp(r, 0.02 * mRadius, mRadius);
}

bool Tool::Contains(double r, double z) const
{
    return r <= RadiusAt(z);
}

std::vector<float> Tool::ProfileAbove(double height, double grow) const
{
    // the heights where the radius changes its slope, the spheres sampled
    std::vector<double> zs = {height, mMaxZ};
    for (const Frustum& f : mFrustums) {
        zs.push_back(f.z0);
        zs.push_back(f.z1);
    }
    for (const Sphere& s : mSpheres) {
        for (int i = 0; i <= 64; i++) {
            zs.push_back(s.zc - s.radius + 2 * s.radius * i / 64);
        }
    }
    std::sort(zs.begin(), zs.end());
    std::vector<float> profile;
    double last = Inf;
    for (auto it = zs.rbegin(); it != zs.rend(); ++it) {
        const double z = *it;
        if (z < height - 1e-12 || z > mMaxZ + 1e-12 || last - z < 1e-9) {
            continue;
        }
        last = z;
        // both sides of a step of the radius
        const double above = RadiusAt(std::min(mMaxZ, z + 1e-9));
        const double below = z > height ? RadiusAt(z - 1e-9) : above;
        profile.push_back((float)(std::max(0.0, above) + grow));
        profile.push_back((float)z);
        if (std::abs(above - below) > 1e-7) {
            profile.push_back((float)(std::max(0.0, below) + grow));
            profile.push_back((float)z);
        }
    }
    return profile;
}

// ---------------------------------------------------------------------------------- normals

uint32_t PackNormal(const float n[3])
{
    float x = n[0], y = n[1];
    const float z = n[2];
    const float sum = std::abs(x) + std::abs(y) + std::abs(z);
    if (!(sum > 0)) {
        return NoNormal;
    }
    x /= sum;
    y /= sum;
    if (z < 0) {
        const float ox = x, oy = y;
        x = (1 - std::abs(oy)) * (ox >= 0 ? 1.0f : -1.0f);
        y = (1 - std::abs(ox)) * (oy >= 0 ? 1.0f : -1.0f);
    }
    // an even scale: 0, -1 and 1 are exact, so faces along the axes stay exact
    const auto u = (uint32_t)((x * 0.5f + 0.5f) * 1022 + 0.5f);
    const auto v = (uint32_t)((y * 0.5f + 0.5f) * 1022 + 0.5f);
    return (v << 10) | u;
}

void UnpackNormal(uint32_t packed, float n[3])
{
    if (!HasNormal(packed)) {
        n[0] = n[1] = 0;
        n[2] = 1;
        return;
    }
    float x = (float)(packed & 1023) / 1022 * 2 - 1;
    float y = (float)((packed >> 10) & 1023) / 1022 * 2 - 1;
    const float z = 1 - std::abs(x) - std::abs(y);
    if (z < 0) {
        const float ox = x, oy = y;
        x = (1 - std::abs(oy)) * (ox >= 0 ? 1.0f : -1.0f);
        y = (1 - std::abs(ox)) * (oy >= 0 ? 1.0f : -1.0f);
    }
    const float len = std::sqrt(x * x + y * y + z * z);
    n[0] = x / len;
    n[1] = y / len;
    n[2] = z / len;
}

// ----------------------------------------------------------------------------------- Sweep

double ParallelogramDistance(
    const double p[3],
    const double base[3],
    const double e1[3],
    const double e2[3],
    int axis
)
{
    double w[3], a[3], b[3];
    for (int k = 0; k < 3; k++) {
        const bool keep = k != axis;
        w[k] = keep ? p[k] - base[k] : 0;
        a[k] = keep ? e1[k] : 0;
        b[k] = keep ? e2[k] : 0;
    }
    const double aa = Dot(a, a), bb = Dot(b, b), ab = Dot(a, b);
    const double wa = Dot(w, a), wb = Dot(w, b);
    const double det = aa * bb - ab * ab;
    // inside the parallelogram: the distance to its plane (zero in 2D)
    if (det > 1e-12 * (aa * bb + 1e-30)) {
        const double x = (wa * bb - wb * ab) / det, y = (wb * aa - wa * ab) / det;
        if (x >= 0 && x <= 1 && y >= 0 && y <= 1) {
            double e[3];
            for (int k = 0; k < 3; k++) {
                e[k] = w[k] - x * a[k] - y * b[k];
            }
            return std::sqrt(Dot(e, e));
        }
    }
    // otherwise the closest of its edges
    auto segment = [&](const double o[3], const double d[3]) {
        double q[3];
        for (int k = 0; k < 3; k++) {
            q[k] = w[k] - o[k];
        }
        const double dd = Dot(d, d);
        const double t = dd > 0 ? std::clamp(Dot(q, d) / dd, 0.0, 1.0) : 0;
        double e[3];
        for (int k = 0; k < 3; k++) {
            e[k] = q[k] - t * d[k];
        }
        return Dot(e, e);
    };
    const double zero[3] = {0, 0, 0};
    double best = std::min(segment(zero, a), segment(zero, b));
    best = std::min(best, segment(a, b));
    best = std::min(best, segment(b, a));
    return std::sqrt(best);
}

double ArcDistance(const Motion& m, double x, double y, double& t)
{
    const double dx = x - m.center[0], dy = y - m.center[1];
    const double rho = std::sqrt(dx * dx + dy * dy);
    if (rho > 1e-12) {
        // the angle of the point in the parametrization of the arc
        const double a = std::atan2(-dx, dy);
        double delta = a - m.startAngle;
        const double twoPi = 2 * M_PI;
        if (m.sweep >= 0) {
            delta = std::fmod(std::fmod(delta, twoPi) + twoPi, twoPi);
        }
        else {
            delta = -std::fmod(std::fmod(-delta, twoPi) + twoPi, twoPi);
        }
        const double tt = m.sweep != 0 ? delta / m.sweep : 2;
        if (tt >= 0 && tt <= 1) {
            t = tt;
            return std::abs(rho - m.radius);
        }
    }
    // the closer end
    auto end = [&](double te) {
        const double a = m.startAngle + te * m.sweep;
        const double ex = m.center[0] - std::sin(a) * m.radius - x;
        const double ey = m.center[1] + std::cos(a) * m.radius - y;
        return std::sqrt(ex * ex + ey * ey);
    };
    const double d0 = end(0), d1 = end(1);
    t = d0 <= d1 ? 0 : 1;
    return std::min(d0, d1);
}

ArcGeometry::ArcGeometry(const Motion& m)
{
    cx = m.center[0];
    cy = m.center[1];
    radius = m.radius;
    const double a0 = m.startAngle, a1 = m.startAngle + m.sweep;
    d0x = -std::sin(a0);
    d0y = std::cos(a0);
    d1x = -std::sin(a1);
    d1y = std::cos(a1);
    x0 = cx + radius * d0x;
    y0 = cy + radius * d0y;
    x1 = cx + radius * d1x;
    y1 = cy + radius * d1y;
    side = m.sweep >= 0 ? 1 : -1;
}

double CuttingHeight(const Motion& motion, const double lo[3], const double hi[3])
{
    // The highest point of the box along the axis, relative to the tip: max over the corners of
    // (corner - tip(t)) . axis(t). Sampled in time, with a margin for the change of the function
    // between samples.
    double a0[3] = {motion.a0[0], motion.a0[1], motion.a0[2]};
    double a1[3] = {motion.a1[0], motion.a1[1], motion.a1[2]};
    Normalize(a0);
    Normalize(a1);
    const double turn = std::acos(std::clamp(Dot(a0, a1), -1.0, 1.0));
    const int samples = turn > 1e-9 ? 64 : 1;
    double d[3], diag = 0, len = 0;
    for (int k = 0; k < 3; k++) {
        d[k] = motion.p1[k] - motion.p0[k];
        len += d[k] * d[k];
        diag += (hi[k] - lo[k]) * (hi[k] - lo[k]);
    }
    len = std::sqrt(len);
    diag = std::sqrt(diag);
    double best = -Inf;
    double farthest = 0;
    for (int i = 0; i <= samples; i++) {
        const double t = samples > 1 ? (double)i / samples : (double)i;
        if (samples == 1 && i == 1) {
            // fixed axis: the ends are the extremes
        }
        double tip[3], axis[3];
        for (int k = 0; k < 3; k++) {
            tip[k] = motion.p0[k] + t * d[k];
            axis[k] = a0[k] + t * (a1[k] - a0[k]);
        }
        Normalize(axis);
        for (int c = 0; c < 8; c++) {
            double h = 0, dist = 0;
            for (int k = 0; k < 3; k++) {
                const double corner = c & (1 << k) ? hi[k] : lo[k];
                h += (corner - tip[k]) * axis[k];
                dist += (corner - tip[k]) * (corner - tip[k]);
            }
            best = std::max(best, h);
            farthest = std::max(farthest, std::sqrt(dist));
        }
    }
    if (samples > 1) {
        // |d/dt| <= |d| + distance * turn rate (normalized interpolation turns at most 2x faster)
        best += (len + farthest * 2 * turn) / samples;
    }
    return best + 1e-6;
}

Sweep::Sweep(const Motion& motion)
    : mMotion(motion)
{
    Normalize(mMotion.a0);
    Normalize(mMotion.a1);
    const Tool& tool = *mMotion.tool;
    const double cut = mMotion.cutHeight;

    // the primitives below the cutting height
    for (size_t i = 0; i < tool.Spheres().size(); i++) {
        const Tool::Sphere& s = tool.Spheres()[i];
        if (s.zc - s.radius >= cut) {
            continue;
        }
        Prim p {};
        p.sphere = true;
        p.index = (int)i;
        p.zc = s.zc;
        p.radius = s.radius;
        p.zm = s.zc;
        p.bound = s.radius;
        mPrims.push_back(p);
    }
    for (size_t i = 0; i < tool.Frustums().size(); i++) {
        const Tool::Frustum& f = tool.Frustums()[i];
        if (f.z0 >= cut) {
            continue;
        }
        Prim p {};
        p.sphere = false;
        p.index = p.index1 = (int)i;
        p.z0 = f.z0;
        p.r0 = f.r0;
        p.z1 = std::min(f.z1, cut);
        p.r1 = f.r0 + (p.z1 - f.z0) * (f.r1 - f.r0) / (f.z1 - f.z0);
        p.zm = 0.5 * (p.z0 + p.z1);
        p.bound = std::sqrt(
            std::max(p.r0, p.r1) * std::max(p.r0, p.r1) + 0.25 * (p.z1 - p.z0) * (p.z1 - p.z0)
        );
        mPrims.push_back(p);
    }
    // merge frustums continuing each other in a straight line, like a shank over the flutes
    for (size_t i = 0; i + 1 < mPrims.size();) {
        Prim& a = mPrims[i];
        const Prim& b = mPrims[i + 1];
        if (!a.sphere && !b.sphere && std::abs(a.z1 - b.z0) < 1e-12 && std::abs(a.r1 - b.r0) < 1e-12) {
            const double ma = (a.r1 - a.r0) / (a.z1 - a.z0), mb = (b.r1 - b.r0) / (b.z1 - b.z0);
            const Tool::Frustum& fa = tool.Frustums()[a.index];
            if (std::abs(ma - mb) < 1e-9 && !fa.sharp1) {
                // the normals of the merged frustum come from the first one: equal on a line
                a.z1 = b.z1;
                a.r1 = b.r1;
                a.index1 = b.index1;
                a.zm = 0.5 * (a.z0 + a.z1);
                a.bound = std::sqrt(
                    std::max(a.r0, a.r1) * std::max(a.r0, a.r1) + 0.25 * (a.z1 - a.z0) * (a.z1 - a.z0)
                );
                mPrims.erase(mPrims.begin() + i + 1);
                continue;
            }
        }
        i++;
    }
    if (mPrims.empty()) {
        for (int k = 0; k < 3; k++) {
            mLo[k] = 1;
            mHi[k] = -1;
        }
        return;
    }

    const double cosTurn = std::clamp(Dot(mMotion.a0, mMotion.a1), -1.0, 1.0);
    const double turn = std::acos(cosTurn);
    mTurning = turn > 1e-9;
    if (!mTurning) {
        // one piece for the whole motion
        mPieces.resize(1);
        Piece& p = mPieces[0];
        for (int k = 0; k < 3; k++) {
            p.p0[k] = mMotion.p0[k];
            p.d[k] = mMotion.p1[k] - mMotion.p0[k];
            p.a[k] = mMotion.a0[k];
        }
        Frame(p.a, p.e1, p.e2);
        mBounds.resize(mPrims.size());
        for (size_t j = 0; j < mPrims.size(); j++) {
            mBounds[j] = BoundOf(p, mPrims[j]);
        }
        if (mMotion.arc) {
            mArc = ArcGeometry(mMotion);
            // the box of the arc: its ends and the extremes of its circle within it
            double lo[2] = {1e300, 1e300}, hi[2] = {-1e300, -1e300};
            auto add = [&](double a) {
                const double x = mMotion.center[0] - std::sin(a) * mMotion.radius;
                const double y = mMotion.center[1] + std::cos(a) * mMotion.radius;
                lo[0] = std::min(lo[0], x);
                hi[0] = std::max(hi[0], x);
                lo[1] = std::min(lo[1], y);
                hi[1] = std::max(hi[1], y);
            };
            add(mMotion.startAngle);
            add(mMotion.startAngle + mMotion.sweep);
            for (int q = -8; q <= 8; q++) {
                const double a = q * M_PI / 2;
                const double t = (a - mMotion.startAngle) / mMotion.sweep;
                if (mMotion.sweep != 0 && t > 0 && t < 1) {
                    add(a);
                }
            }
            for (size_t j = 0; j < mPrims.size(); j++) {
                const Prim& prim = mPrims[j];
                const double r = PrimRadius(prim) + 1e-6;
                Bound& b = mBounds[j];
                for (int k = 0; k < 2; k++) {
                    b.lo[k] = std::nextafter((float)(lo[k] - r), -1e30f);
                    b.hi[k] = std::nextafter((float)(hi[k] + r), 1e30f);
                }
            }
        }
    }
    else {
        // Motions turning the axis are swept in pieces, every primitive on its own: in a piece,
        // a primitive turns about its center by a small angle while the center moves on a chord
        // of its path. Points at a distance 'reach' from the center move by reach * angle / 2
        // from where the piece has them, the chord deviates by height * angle^2 / 8 from the
        // path of the center. A sphere doesn't change turning about its center. Long frustums
        // are split, so their pieces are short and their bounds tight.
        std::vector<Prim> split;
        for (const Prim& prim : mPrims) {
            if (prim.sphere) {
                split.push_back(prim);
                continue;
            }
            const double rmax = std::max(prim.r0, prim.r1);
            const double height = prim.z1 - prim.z0;
            const int parts = std::clamp((int)std::ceil(height / std::max(2 * rmax, 0.5)), 1, 64);
            for (int i = 0; i < parts; i++) {
                Prim sub = prim;
                sub.z0 = prim.z0 + height * i / parts;
                sub.z1 = prim.z0 + height * (i + 1) / parts;
                sub.r0 = prim.r0 + (sub.z0 - prim.z0) * (prim.r1 - prim.r0) / height;
                sub.r1 = prim.r0 + (sub.z1 - prim.z0) * (prim.r1 - prim.r0) / height;
                sub.zm = 0.5 * (sub.z0 + sub.z1);
                const double r = std::max(sub.r0, sub.r1);
                sub.bound = std::sqrt(r * r + 0.25 * (sub.z1 - sub.z0) * (sub.z1 - sub.z0));
                // only the last part has the top rim of the frustum
                if (i + 1 < parts) {
                    sub.index1 = -1;
                }
                split.push_back(sub);
            }
        }
        mPrims.swap(split);
        for (int j = 0; j < (int)mPrims.size(); j++) {
            const Prim& prim = mPrims[j];
            const double tol = std::max(1e-6, mMotion.tolerance);
            const double chord = std::sqrt(8 * tol / std::max(std::abs(prim.zm), 1e-3));
            const double rotation = prim.sphere ? 1e30 : 2 * tol / std::max(prim.bound, 1e-3);
            const int pieces = std::clamp((int)std::ceil(turn / chord + turn / rotation), 1, 1 << 16);
            for (int i = 0; i < pieces; i++) {
                const double t0 = (double)i / pieces, t1 = (double)(i + 1) / pieces;
                const Piece p = PieceOf(prim, t0, t1);
                mPieces.push_back(p);
                mEntryPrim.push_back(j);
                mEntryT0.push_back(t0);
                mEntryT1.push_back(t1);
                mBounds.push_back(BoundOf(p, prim));
            }
        }
    }
    for (int k = 0; k < 3; k++) {
        mLo[k] = Inf;
        mHi[k] = -Inf;
    }
    for (const Bound& b : mBounds) {
        for (int k = 0; k < 3; k++) {
            mLo[k] = std::min(mLo[k], (double)b.lo[k]);
            mHi[k] = std::max(mHi[k], (double)b.hi[k]);
        }
    }
    const int np = (int)mPrims.size();
    const int pieces = (int)mPieces.size();

    // search orders of the primitives
    mByLow.resize(np);
    mByHigh.resize(np);
    for (int j = 0; j < np; j++) {
        mByLow[j] = mByHigh[j] = j;
    }
    std::sort(mByLow.begin(), mByLow.end(), [&](int a, int b) {
        return PrimLow(mPrims[a]) < PrimLow(mPrims[b]);
    });
    std::sort(mByHigh.begin(), mByHigh.end(), [&](int a, int b) {
        return PrimHigh(mPrims[a]) > PrimHigh(mPrims[b]);
    });
    mByLowRadius.resize(np);
    mByLowTop.resize(np);
    mByHighRadius.resize(np);
    for (int j = 0; j < np; j++) {
        const Prim& lowP = mPrims[mByLow[j]];
        mByLowRadius[j] = std::max(j ? mByLowRadius[j - 1] : 0.0, PrimRadius(lowP));
        mByLowTop[j] = std::max(j ? mByLowTop[j - 1] : -Inf, PrimHigh(lowP));
        mByHighRadius[j] = std::max(j ? mByHighRadius[j - 1] : 0.0, PrimRadius(mPrims[mByHigh[j]]));
    }

    if (mTurning) {
        mNodes.reserve(2 * pieces);
        BuildNode(0, pieces);
    }
    else {
        // the capsule around the axis segment of all primitives, from the middle of their heights
        double zlo = Inf, zhi = -Inf, rmax = 0;
        for (const Prim& prim : mPrims) {
            zlo = std::min(zlo, prim.sphere ? prim.zc - prim.radius : prim.z0);
            zhi = std::max(zhi, prim.sphere ? prim.zc + prim.radius : prim.z1);
            rmax = std::max(rmax, prim.sphere ? prim.radius : std::max(prim.r0, prim.r1));
        }
        const Piece& p = mPieces[0];
        for (int k = 0; k < 3; k++) {
            mSlabBase[k] = p.p0[k] + zlo * p.a[k];
            mSlabE1[k] = p.d[k];
            mSlabE2[k] = (zhi - zlo) * p.a[k];
        }
        mSlabRadius = mMotion.arc ? -1 : rmax + 1e-6;
    }
}

Sweep::Piece Sweep::PieceOf(const Prim& prim, double t0, double t1) const
{
    double tip0[3], tip1[3], axis0[3], axis1[3], tipMid[3], axisMid[3];
    PoseAt(t0, tip0, axis0);
    PoseAt(t1, tip1, axis1);
    PoseAt(0.5 * (t0 + t1), tipMid, axisMid);
    Piece p;
    for (int k = 0; k < 3; k++) {
        // the center at both ends, with the axis of the middle
        const double c0 = tip0[k] + prim.zm * axis0[k];
        const double c1 = tip1[k] + prim.zm * axis1[k];
        p.a[k] = axisMid[k];
        p.p0[k] = c0 - prim.zm * axisMid[k];
        p.d[k] = c1 - c0;
    }
    Frame(p.a, p.e1, p.e2);
    return p;
}

Sweep::Bound Sweep::BoundOf(const Piece& p, const Prim& prim)
{
    Bound b;
    const double rmax = prim.sphere ? prim.radius : std::max(prim.r0, prim.r1);
    for (int k = 0; k < 3; k++) {
        b.c0[k] = p.p0[k] + prim.zm * p.a[k];
        b.c1[k] = b.c0[k] + p.d[k];
        const double lo = std::min(b.c0[k], b.c1[k]), hi = std::max(b.c0[k], b.c1[k]);
        // a frustum is a cylinder of its largest radius around its axis segment
        double ext;
        if (prim.sphere) {
            ext = prim.radius;
        }
        else {
            const double half = 0.5 * (prim.z1 - prim.z0);
            ext = std::abs(p.a[k]) * half + rmax * std::sqrt(std::max(0.0, 1 - p.a[k] * p.a[k]));
        }
        ext += 1e-6;
        // float rounding outwards
        b.lo[k] = std::nextafter((float)(lo - ext), -1e30f);
        b.hi[k] = std::nextafter((float)(hi + ext), 1e30f);
    }
    return b;
}

bool Sweep::MayTouch(const double pt[3], double margin) const
{
    if (mMotion.arc) {
        // within the height of the tool, and its radius from the arc
        double zlo = Inf, zhi = -Inf, rmax = 0;
        for (const Prim& prim : mPrims) {
            zlo = std::min(zlo, PrimLow(prim));
            zhi = std::max(zhi, PrimHigh(prim));
            rmax = std::max(rmax, PrimRadius(prim));
        }
        const double h = (pt[2] - mMotion.p0[2]) * mMotion.a0[2];
        if (h < zlo - margin || h > zhi + margin) {
            return false;
        }
        return mArc.Distance(pt[0], pt[1]) <= rmax + margin;
    }
    if (mSlabRadius < 0) {
        return true;
    }
    // tool frame of the cylinder: the slab base is the axis at its lowest height
    const Piece& p = mPieces[0];
    double w[3];
    for (int k = 0; k < 3; k++) {
        w[k] = pt[k] - mSlabBase[k];
    }
    const double height = std::sqrt(Dot(mSlabE2, mSlabE2));
    const double zeta = Dot(w, p.a), dz = Dot(p.d, p.a);
    // times when the height of the point is within the cylinder
    double t0 = 0, t1 = 1;
    if (std::abs(dz) > 1e-12) {
        double a = (zeta - height - margin) / dz, b = (zeta + margin) / dz;
        if (a > b) {
            std::swap(a, b);
        }
        t0 = std::max(t0, a);
        t1 = std::min(t1, b);
        if (t0 > t1) {
            return false;
        }
    }
    else if (zeta < -margin || zeta > height + margin) {
        return false;
    }
    // distance across the axis from the path of the axis during those times
    double r[3], dp[3];
    for (int k = 0; k < 3; k++) {
        r[k] = w[k] - zeta * p.a[k];
        dp[k] = p.d[k] - dz * p.a[k];
    }
    const double dd = Dot(dp, dp);
    const double t = dd > 0 ? std::clamp(Dot(r, dp) / dd, t0, t1) : t0;
    double e[3];
    for (int k = 0; k < 3; k++) {
        e[k] = r[k] - t * dp[k];
    }
    const double reach = mSlabRadius + margin;
    return Dot(e, e) <= reach * reach;
}

double Sweep::PrimLow(const Prim& p)
{
    return p.sphere ? p.zc - p.radius : p.z0;
}

double Sweep::PrimHigh(const Prim& p)
{
    return p.sphere ? p.zc + p.radius : p.z1;
}

double Sweep::PrimRadius(const Prim& p)
{
    return p.sphere ? p.radius : std::max(p.r0, p.r1);
}

int Sweep::BuildNode(int first, int count)
{
    const int index = (int)mNodes.size();
    mNodes.push_back({});
    float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
    for (int i = first; i < first + count; i++) {
        const Bound& b = mBounds[i];
        for (int k = 0; k < 3; k++) {
            lo[k] = std::min(lo[k], b.lo[k]);
            hi[k] = std::max(hi[k], b.hi[k]);
        }
    }
    Node n {};
    for (int k = 0; k < 3; k++) {
        n.lo[k] = lo[k];
        n.hi[k] = hi[k];
    }
    // pieces of one primitive: the primitive turning about its center over their times, with
    // the axis of the middle, is within reach * angle / 2 of that, its center within
    // height * angle^2 / 8 of the chord
    n.prim = mEntryPrim[first];
    for (int i = first; i < first + count && n.prim >= 0; i++) {
        if (mEntryPrim[i] != n.prim) {
            n.prim = -1;
        }
    }
    if (n.prim >= 0) {
        const Prim& prim = mPrims[n.prim];
        const double t0 = mEntryT0[first], t1 = mEntryT1[first + count - 1];
        n.pose = PieceOf(prim, t0, t1);
        double a0[3], a1[3], unused[3];
        PoseAt(t0, unused, a0);
        PoseAt(t1, unused, a1);
        const double angle = std::acos(std::clamp(Dot(a0, a1), -1.0, 1.0));
        n.grow = (prim.sphere ? 0 : prim.bound * angle / 2) + std::abs(prim.zm) * angle * angle / 8
            + 1e-6;
    }
    if (count <= 4) {
        n.first = first;
        n.count = count;
        mNodes[index] = n;
        return index;
    }
    const int half = count / 2;
    n.count = -1;
    mNodes[index] = n;
    const int left = BuildNode(first, half);
    const int right = BuildNode(first + half, count - half);
    mNodes[index].first = left;
    mNodes[index].right = right;
    return index;
}

void Sweep::PoseAt(double t, double tip[3], double axisDir[3]) const
{
    for (int k = 0; k < 3; k++) {
        tip[k] = mMotion.p0[k] + t * (mMotion.p1[k] - mMotion.p0[k]);
        axisDir[k] = mMotion.a0[k] + t * (mMotion.a1[k] - mMotion.a0[k]);
    }
    if (mMotion.arc) {
        const double a = mMotion.startAngle + t * mMotion.sweep;
        tip[0] = mMotion.center[0] - std::sin(a) * mMotion.radius;
        tip[1] = mMotion.center[1] + std::cos(a) * mMotion.radius;
        tip[2] = mMotion.p0[2];
    }
    Normalize(axisDir);
}

bool Sweep::SphereLine(const Piece& p, const Prim& sp, int axis, const double o[3], Interval& out) const
{
    const double R = sp.radius;
    // capsule: centers c0 + tau * d
    double w[3];
    for (int k = 0; k < 3; k++) {
        w[k] = o[k] - (p.p0[k] + sp.zc * p.a[k]);
    }
    const double dd = Dot(p.d, p.d);
    double lo = Inf, hi = -Inf;
    // |c + s u|^2 = R^2 with u the unit axis
    auto sphere = [&](const double c[3]) {
        const double ca = c[axis];
        const double disc = ca * ca - Dot(c, c) + R * R;
        if (disc >= 0) {
            const double sq = std::sqrt(disc);
            lo = std::min(lo, -ca - sq);
            hi = std::max(hi, -ca + sq);
        }
    };
    sphere(w);
    if (dd > 1e-24) {
        const double w1[3] = {w[0] - p.d[0], w[1] - p.d[1], w[2] - p.d[2]};
        sphere(w1);
        // cylinder part: |w_perp + s u_perp|^2 = R^2, with 0 <= (w + s u).d / dd <= 1
        const double wd = Dot(w, p.d), ud = p.d[axis];
        const double uu = 1 - ud * ud / dd;  // |u_perp|^2
        if (uu > 1e-14) {
            const double wu = w[axis] - wd * ud / dd;  // w_perp . u_perp
            const double ww = Dot(w, w) - wd * wd / dd;
            const double disc = wu * wu - uu * (ww - R * R);
            if (disc >= 0) {
                const double sq = std::sqrt(disc);
                double s0 = (-wu - sq) / uu, s1 = (-wu + sq) / uu;
                if (std::abs(ud) > 1e-14) {
                    double a = -wd / ud, b = (dd - wd) / ud;
                    if (a > b) {
                        std::swap(a, b);
                    }
                    s0 = std::max(s0, a);
                    s1 = std::min(s1, b);
                }
                else if (wd < 0 || wd > dd) {
                    s0 = Inf;
                    s1 = -Inf;
                }
                if (s0 <= s1) {
                    lo = std::min(lo, s0);
                    hi = std::max(hi, s1);
                }
            }
        }
    }
    if (lo > hi) {
        return false;
    }
    const double base = o[axis];
    out.lo = {base + lo, 0, 0, SphereSurface, 0};
    out.hi = {base + hi, 0, 0, SphereSurface, 0};
    return true;
}

bool Sweep::FrustumLine(const Piece& p, const Prim& f, int axis, const double o[3], Interval& out) const
{
    // The points (s, t) of the line X(s) = o + s u inside the frustum at time t of the piece form
    // a convex region: t in [0, 1], z in [z0, z1] (two strips) and inside the cone, bounded by a
    // conic. The extremes of s are at corners of the strips, where their lines meet the conic, or
    // where the conic is tangent to the lines of constant s.
    const double z0 = f.z0, z1 = f.z1;
    const double r0 = f.r0, r1 = f.r1;
    const double m = (r1 - r0) / (z1 - z0);

    // tool frame coordinates as linear forms in (s, t): W(s, t) = w + s u - t d
    struct Lin
    {
        double a, b, c;
        double At(double s, double t) const
        {
            return a * s + b * t + c;
        }
    };
    const double q[3] = {o[0] - p.p0[0], o[1] - p.p0[1], o[2] - p.p0[2]};
    const double dx = Dot(p.d, p.e1), dy = Dot(p.d, p.e2), dz = Dot(p.d, p.a);
    const Lin X {p.e1[axis], -dx, Dot(q, p.e1)};
    const Lin Y {p.e2[axis], -dy, Dot(q, p.e2)};
    const Lin Z {p.a[axis], -dz, Dot(q, p.a)};
    const Lin Rho {m * Z.a, m * Z.b, r0 + m * (Z.c - z0)};
    // scale of the parameter s over the region, and where it is
    const double rMax = std::max(r0, r1);
    const double scale = std::sqrt(dx * dx + dy * dy + dz * dz) + 2 * (rMax + (z1 - z0)) + 1;
    const double sRef = -(X.a * (X.c + 0.5 * X.b) + Y.a * (Y.c + 0.5 * Y.b) + Z.a * (Z.c + 0.5 * Z.b));

    double lo = Inf, hi = -Inf;
    int loKind = 0, hiKind = 0;
    double loT = 0, hiT = 0;
    // a candidate counts if it is inside the region, evaluated on the coordinates themselves
    auto consider = [&](double s, double t, int kind) {
        if (!(t >= -1e-9 && t <= 1 + 1e-9)) {
            return;
        }
        const double z = Z.At(s, t);
        if (!(z >= z0 - 1e-7 && z <= z1 + 1e-7)) {
            return;
        }
        const double rho = r0 + m * (std::clamp(z, z0, z1) - z0);
        const double x = X.At(s, t), y = Y.At(s, t);
        const double dist = std::sqrt(x * x + y * y);
        if (dist > rho + 1e-7) {
            return;
        }
        if ((kind == CapBottom || kind == CapTop) && dist > rho - 1e-7) {
            kind = kind == CapBottom ? Rim0 : Rim1;
        }
        t = std::clamp(t, 0.0, 1.0);
        if (s < lo) {
            lo = s;
            loKind = kind;
            loT = t;
        }
        if (s > hi) {
            hi = s;
            hiKind = kind;
            hiT = t;
        }
    };
    // where the line (s, t) = P + l V meets the cone
    auto onCone = [&](double ps, double pt, double vs, double vt, int kind) {
        const double xp = X.At(ps, pt), xv = X.a * vs + X.b * vt;
        const double yp = Y.At(ps, pt), yv = Y.a * vs + Y.b * vt;
        const double rp = Rho.At(ps, pt), rv = Rho.a * vs + Rho.b * vt;
        double roots[2];
        const int nr = SolveQuadratic(
            xv * xv + yv * yv - rv * rv,
            xv * xp + yv * yp - rv * rp,
            xp * xp + yp * yp - rp * rp,
            roots
        );
        for (int i = 0; i < nr; i++) {
            const double l = roots[i];
            if (rp + l * rv >= -1e-9) {  // the nappe of positive radius
                consider(ps + l * vs, pt + l * vt, kind);
            }
        }
    };
    // the line a s + b t = c through the region, parametrized along its better conditioned
    // direction
    auto lineOnCone = [&](double a, double b, double c, int kind) {
        if (std::abs(b) * scale >= std::abs(a)) {
            if (b == 0) {
                return;
            }
            // t as a function of s, around sRef
            onCone(sRef, (c - a * sRef) / b, 1, -a / b, kind);
        }
        else {
            // s as a function of t, around t = 0.5
            onCone((c - b * 0.5) / a, 0.5, -b / a, 1, kind);
        }
    };

    // (i) corners: t in {0, 1} and z in {z0, z1}
    if (std::abs(Z.a) * scale > std::abs(Z.b) * 1e-9 && Z.a != 0) {
        for (double t : {0.0, 1.0}) {
            consider((z0 - Z.c - Z.b * t) / Z.a, t, CapBottom);
            consider((z1 - Z.c - Z.b * t) / Z.a, t, CapTop);
        }
    }
    // (ii) the lines t = 0 and t = 1 on the cone
    lineOnCone(0, 1, 0, ConeSurface);
    lineOnCone(0, 1, 1, ConeSurface);
    // the rims: z = z0, z = z1 on the cone
    if (std::abs(Z.a) + std::abs(Z.b) / scale > 1e-12) {
        lineOnCone(Z.a, Z.b, z0 - Z.c, Rim0);
        lineOnCone(Z.a, Z.b, z1 - Z.c, Rim1);
    }
    // (iii) where the cone is tangent to s = constant: dQ/dt = 0, a line in (s, t)
    const double qst = X.a * X.b + Y.a * Y.b - Rho.a * Rho.b;
    const double qtt = X.b * X.b + Y.b * Y.b - Rho.b * Rho.b;
    const double qt = X.b * X.c + Y.b * Y.c - Rho.b * Rho.c;
    if (std::abs(qtt) + std::abs(qst) * scale > 1e-14 * (rMax * rMax + 1)) {
        lineOnCone(qst, qtt, -qt, ConeSurface);
    }
    if (lo > hi) {
        return false;
    }
    // A cap moving in its plane sweeps a face of that plane, also where the line leaves it at
    // the rim
    if (std::abs(Z.b) <= 1e-9 * scale) {
        for (int* kind : {&loKind, &hiKind}) {
            if (*kind == Rim0) {
                *kind = CapBottom;
            }
            else if (*kind == Rim1) {
                *kind = CapTop;
            }
        }
    }
    const double base = o[axis];
    out.lo = {base + lo, 0, 0, loKind, loT};
    out.hi = {base + hi, 0, 0, hiKind, hiT};
    return true;
}


void Sweep::ArcDiskLine(int axis, const double o[3], double r, int prim, std::vector<Interval>& found) const
{
    // In the plane of the line across the axis: the points within r of the arc are the annulus
    // around its circle in the wedge of the arc, and the disks around its ends. The line is
    // q(s) = (o[0], o[1]) + s e, s the world coordinate along the axis (0 or 1).
    const double cx = mMotion.center[0], cy = mMotion.center[1], R = mMotion.radius;
    const double ex = axis == 0 ? 1 : 0, ey = axis == 1 ? 1 : 0;
    const double ox = axis == 0 ? 0 : o[0], oy = axis == 1 ? 0 : o[1];
    std::pair<double, double> pieces[8];
    int count = 0;
    // the line in a disk
    auto disk = [&](double x, double y, double radius, double& s0, double& s1) {
        const double wx = ox - x, wy = oy - y;
        const double b = wx * ex + wy * ey;
        const double c = wx * wx + wy * wy - radius * radius;
        const double disc = b * b - c;
        if (disc < 0) {
            return false;
        }
        const double sq = std::sqrt(disc);
        s0 = -b - sq;
        s1 = -b + sq;
        return true;
    };
    double s0, s1;
    for (double t : {0.0, 1.0}) {
        double tip[3], unused[3];
        PoseAt(t, tip, unused);
        if (disk(tip[0], tip[1], r, s0, s1)) {
            pieces[count++] = {s0, s1};
        }
    }
    // the wedge: on the inner side of the rays to both ends
    double w0 = -Inf, w1 = Inf;
    for (int end = 0; end < 2; end++) {
        const double a = mMotion.startAngle + end * mMotion.sweep;
        // direction of the end from the center, and the side of the wedge: the arc turns
        // counterclockwise for a positive sweep
        const double dx = -std::sin(a), dy = std::cos(a);
        const double side = (mMotion.sweep >= 0 ? 1 : -1) * (end == 0 ? 1 : -1);
        // side * cross(d, q - c) >= 0
        const double alpha = side * (dx * (oy - cy) - dy * (ox - cx));
        const double beta = side * (dx * ey - dy * ex);
        if (std::abs(beta) < 1e-15) {
            if (alpha < 0) {
                w0 = Inf;
                w1 = -Inf;
            }
            continue;
        }
        const double sb = -alpha / beta;
        if (beta > 0) {
            w0 = std::max(w0, sb);
        }
        else {
            w1 = std::min(w1, sb);
        }
    }
    double a0, a1;
    if (w0 <= w1 && disk(cx, cy, R + r, a0, a1)) {
        a0 = std::max(a0, w0);
        a1 = std::min(a1, w1);
        double i0, i1;
        if (R > r && disk(cx, cy, R - r, i0, i1)) {
            // minus the inner disk
            if (a0 < std::min(a1, i0)) {
                pieces[count++] = {a0, std::min(a1, i0)};
            }
            if (std::max(a0, i1) < a1) {
                pieces[count++] = {std::max(a0, i1), a1};
            }
        }
        else if (a0 <= a1) {
            pieces[count++] = {a0, a1};
        }
    }
    if (count == 0) {
        return;
    }
    // at most 4 pieces
    for (int i = 1; i < count; i++) {
        for (int j = i; j > 0 && pieces[j] < pieces[j - 1]; j--) {
            std::swap(pieces[j], pieces[j - 1]);
        }
    }
    const Prim& p = mPrims[prim];
    auto endOf = [&](double s) {
        // the time of the closest point is found for the normals, when needed
        return RemovalEnd {s, 0, prim, p.sphere ? SphereSurface : ConeSurface, -1};
    };
    double lo = pieces[0].first, hi = pieces[0].second;
    for (int i = 1; i <= count; i++) {
        if (i < count && pieces[i].first <= hi) {
            hi = std::max(hi, pieces[i].second);
            continue;
        }
        found.push_back({endOf(lo), endOf(hi)});
        if (i < count) {
            lo = pieces[i].first;
            hi = pieces[i].second;
        }
    }
}

void Sweep::LineEntry(
    int e,
    int axis,
    const double o[3],
    double sMin,
    double sMax,
    std::vector<Interval>& found
) const
{
    const int u = (axis + 1) % 3, v = (axis + 2) % 3;
    const Bound& b = mBounds[e];
    if (o[u] < b.lo[u] || o[u] > b.hi[u] || o[v] < b.lo[v] || o[v] > b.hi[v] || b.hi[axis] < sMin
        || b.lo[axis] > sMax) {
        return;
    }
    const int j = mEntryPrim[e];
    const Prim& prim = mPrims[j];
    const double du = b.c1[u] - b.c0[u], dv = b.c1[v] - b.c0[v];
    const double wu = o[u] - b.c0[u], wv = o[v] - b.c0[v];
    const double dd = du * du + dv * dv;
    const double tt = dd > 0 ? std::clamp((wu * du + wv * dv) / dd, 0.0, 1.0) : 0;
    const double eu = wu - tt * du, ev = wv - tt * dv;
    if (eu * eu + ev * ev > prim.bound * prim.bound) {
        return;
    }
    Interval iv;
    const Piece& p = mPieces[e];
    const bool hit = prim.sphere ? SphereLine(p, prim, axis, o, iv)
                                 : FrustumLine(p, prim, axis, o, iv);
    if (!hit || iv.hi.s < sMin || iv.lo.s > sMax) {
        return;
    }
    iv.lo.primitive = iv.hi.primitive = j;
    iv.lo.piece = iv.hi.piece = e;
    found.push_back(iv);
}

void Sweep::LinePiece(
    int pi,
    int axis,
    const double o[3],
    double sMin,
    double sMax,
    std::vector<Interval>& found
) const
{
    const int u = (axis + 1) % 3, v = (axis + 2) % 3;
    const Piece& p = mPieces[pi];
    const int np = (int)mPrims.size();
    const double base = o[axis];

    // Fast exact paths for motions across the tool axis (no motion along it), the most common:
    // lines along the axis and lines across it
    const double dLen2 = Dot(p.d, p.d);
    const double dAlong = Dot(p.d, p.a);
    const bool across = dAlong * dAlong <= 1e-24 * (1 + dLen2);
    const double aAxis = p.a[axis];
    // line along the axis: s = tip(t)[axis] + sigma z
    const bool along = across && std::abs(aAxis) > 1 - 1e-12;
    // line across the axis: the tool frame height of the line is fixed
    const bool lineAcross = across && std::abs(aAxis) < 1e-12;
    double rhoMin = 0, tauMin = 0, zLine = 0;
    if (along && mMotion.arc) {
        // the time of the closest point is found for the normals, when needed
        rhoMin = mArc.Distance(o[0], o[1]);
        tauMin = -1;
    }
    else if (along) {
        // distances of the line from the axis during the motion: the closest approach and the
        // farthest end
        const double du = p.d[u], dv = p.d[v];
        const double wu = o[u] - p.p0[u], wv = o[v] - p.p0[v];
        const double dd = du * du + dv * dv;
        tauMin = dd > 0 ? std::clamp((wu * du + wv * dv) / dd, 0.0, 1.0) : 0;
        const double eu = wu - tauMin * du, ev = wv - tauMin * dv;
        rhoMin = std::sqrt(eu * eu + ev * ev);
        if (rhoMin > mSlabRadius && mSlabRadius >= 0) {
            return;
        }
    }
    if (lineAcross) {
        zLine = (o[0] - p.p0[0]) * p.a[0] + (o[1] - p.p0[1]) * p.a[1] + (o[2] - p.p0[2]) * p.a[2];
        if (mSlabRadius >= 0) {
            const double height = std::sqrt(Dot(mSlabE2, mSlabE2));
            const double zlo = Dot(mSlabBase, p.a) - Dot(p.p0, p.a);
            if (zLine < zlo - 1e-9 || zLine > zlo + height + 1e-9) {
                return;
            }
        }
    }
    // across: the largest cross section at the height of the line
    int bestPrim = -1;
    double bestRadius = -1;
    // the heights of a primitive along a line at a distance from the axis
    auto heights = [&](const Prim& prim, double rho, double& zLo, double& zHi, int& kLo, int& kHi) {
        if (prim.sphere) {
            const double half = std::sqrt(std::max(0.0, prim.radius * prim.radius - rho * rho));
            zLo = prim.zc - half;
            zHi = prim.zc + half;
            kLo = kHi = SphereSurface;
            return;
        }
        const double r0 = prim.r0, r1 = prim.r1;
        zLo = prim.z0;
        zHi = prim.z1;
        kLo = CapBottom;
        kHi = CapTop;
        if (r1 > r0 && rho > r0) {
            // widening upwards: the lowest point at the closest approach on the cone
            zLo = prim.z0 + (rho - r0) * (prim.z1 - prim.z0) / (r1 - r0);
            kLo = ConeSurface;
        }
        else if (r0 > r1 && rho > r1) {
            zHi = prim.z0 + (rho - r0) * (prim.z1 - prim.z0) / (r1 - r0);
            kHi = ConeSurface;
        }
    };
    // an interval along the axis in tool heights, as a removal along the line
    const double tip = p.p0[axis] - base;
    auto alongInterval = [&](int jLo, double zLo, int kLo, int jHi, double zHi, int kHi) {
        RemovalEnd eLo {0, pi, jLo, kLo, tauMin}, eHi {0, pi, jHi, kHi, tauMin};
        if (aAxis > 0) {
            eLo.s = base + tip + zLo;
            eHi.s = base + tip + zHi;
        }
        else {
            eHi.s = base + tip - zLo;
            eLo.s = base + tip - zHi;
            std::swap(eLo, eHi);
            eLo.s = base + tip - zHi;
            eHi.s = base + tip - zLo;
        }
        if (eHi.s >= sMin && eLo.s <= sMax) {
            found.push_back({eLo, eHi});
        }
    };

    if (along && mMotion.tool->SingleAxial()) {
        // Tools without undercuts: one interval, from the lowest point of the primitives at the
        // closest approach to the highest. The primitives by height, from the first one reaching
        // that far from the axis, until no further one can be lower (higher).
        const int first = (int)(std::lower_bound(mByLowRadius.begin(), mByLowRadius.end(), rhoMin)
                                - mByLowRadius.begin());
        double bestLo = Inf, bestHi = -Inf;
        int jLo = -1, jHi = -1, kLo = 0, kHi = 0;
        for (int k = first; k < np; k++) {
            const Prim& prim = mPrims[mByLow[k]];
            if (PrimLow(prim) >= bestLo) {
                break;
            }
            if (PrimRadius(prim) < rhoMin) {
                continue;
            }
            double zLo, zHi;
            int a, b;
            heights(prim, rhoMin, zLo, zHi, a, b);
            if (zLo < bestLo) {
                bestLo = zLo;
                jLo = mByLow[k];
                kLo = a;
            }
        }
        if (jLo < 0) {
            return;
        }
        const int firstHigh = (int)(std::lower_bound(mByHighRadius.begin(), mByHighRadius.end(), rhoMin)
                                    - mByHighRadius.begin());
        for (int k = firstHigh; k < np; k++) {
            const Prim& prim = mPrims[mByHigh[k]];
            if (PrimHigh(prim) <= bestHi) {
                break;
            }
            if (PrimRadius(prim) < rhoMin) {
                continue;
            }
            double zLo, zHi;
            int a, b;
            heights(prim, rhoMin, zLo, zHi, a, b);
            if (zHi > bestHi) {
                bestHi = zHi;
                jHi = mByHigh[k];
                kHi = b;
            }
        }
        alongInterval(jLo, bestLo, kLo, jHi, bestHi, kHi);
        return;
    }
    if (lineAcross) {
        // the primitives containing the height of the line: from the last one starting below it,
        // back while one of them reaches above it
        int k = (int)(std::upper_bound(
                          mByLow.begin(),
                          mByLow.end(),
                          zLine,
                          [&](double z, int j) { return z < PrimLow(mPrims[j]); }
                      )
                      - mByLow.begin())
            - 1;
        for (; k >= 0 && mByLowTop[k] >= zLine; k--) {
            const Prim& prim = mPrims[mByLow[k]];
            double r;
            if (prim.sphere) {
                const double dz = zLine - prim.zc;
                if (std::abs(dz) > prim.radius) {
                    continue;
                }
                r = std::sqrt(prim.radius * prim.radius - dz * dz);
            }
            else {
                if (zLine < prim.z0 || zLine > prim.z1) {
                    continue;
                }
                r = prim.r0 + (zLine - prim.z0) * (prim.r1 - prim.r0) / (prim.z1 - prim.z0);
            }
            if (r > bestRadius) {
                bestRadius = r;
                bestPrim = mByLow[k];
            }
        }
    }

    for (int j = 0; j < np && !lineAcross; j++) {
        const Prim& prim = mPrims[j];
        if (along) {
            if (rhoMin > PrimRadius(prim)) {
                continue;
            }
            double zLo, zHi;
            int kLo, kHi;
            heights(prim, rhoMin, zLo, zHi, kLo, kHi);
            alongInterval(j, zLo, kLo, j, zHi, kHi);
            continue;
        }
        const Bound& b = mBounds[(size_t)pi * np + j];
        if (o[u] < b.lo[u] || o[u] > b.hi[u] || o[v] < b.lo[v] || o[v] > b.hi[v]
            || b.hi[axis] < sMin || b.lo[axis] > sMax) {
            continue;
        }
        // distance from the line to the segment of the center of the primitive, in the plane
        // across the line
        const double du = b.c1[u] - b.c0[u], dv = b.c1[v] - b.c0[v];
        const double wu = o[u] - b.c0[u], wv = o[v] - b.c0[v];
        const double dd = du * du + dv * dv;
        const double tt = dd > 0 ? std::clamp((wu * du + wv * dv) / dd, 0.0, 1.0) : 0;
        const double eu = wu - tt * du, ev = wv - tt * dv;
        if (eu * eu + ev * ev > prim.bound * prim.bound) {
            continue;
        }
        Interval iv;
        const bool hit = prim.sphere ? SphereLine(p, prim, axis, o, iv)
                                     : FrustumLine(p, prim, axis, o, iv);
        if (!hit || iv.hi.s < sMin || iv.lo.s > sMax) {
            continue;
        }
        iv.lo.primitive = iv.hi.primitive = j;
        iv.lo.piece = iv.hi.piece = pi;
        found.push_back(iv);
    }

    if (mMotion.arc) {
        if (lineAcross && bestPrim >= 0 && bestRadius > 0) {
            ArcDiskLine(axis, o, bestRadius, bestPrim, found);
        }
        return;
    }
    if (lineAcross && bestPrim >= 0 && bestRadius > 0) {
        // the disk of the largest cross section swept along the motion: a capsule in the plane
        // of the line
        Prim disk {};
        disk.sphere = true;
        disk.zc = zLine;
        disk.radius = bestRadius;
        Interval iv;
        if (SphereLine(p, disk, axis, o, iv) && iv.hi.s >= sMin && iv.lo.s <= sMax) {
            const Prim& prim = mPrims[bestPrim];
            for (RemovalEnd* e : {&iv.lo, &iv.hi}) {
                e->piece = pi;
                e->primitive = bestPrim;
                if (!prim.sphere) {
                    // the time of the closest approach to the end, for its normal
                    double w[3];
                    for (int k = 0; k < 3; k++) {
                        w[k] = o[k] - p.p0[k];
                    }
                    w[axis] = e->s - p.p0[axis];
                    e->kind = ConeSurface;
                    e->t = dLen2 > 1e-24 ? std::clamp(Dot(w, p.d) / dLen2, 0.0, 1.0) : 0;
                }
            }
            found.push_back(iv);
        }
    }
}

int Sweep::Line(int axis, const double origin[3], double sMin, double sMax, std::vector<Removal>& out) const
{
    const int u = (axis + 1) % 3, v = (axis + 2) % 3;
    // removals that only touch the material don't change it
    if (origin[u] < mLo[u] || origin[u] > mHi[u] || origin[v] < mLo[v] || origin[v] > mHi[v]
        || mHi[axis] <= sMin + 1e-7 || mLo[axis] >= sMax - 1e-7) {
        return 0;
    }
    // origin on the line at s = 0 in world coordinates along the axis
    double o[3] = {origin[0], origin[1], origin[2]};
    o[axis] = 0;

    thread_local std::vector<Interval> found;
    found.clear();
    if (mNodes.empty()) {
        LinePiece(0, axis, o, sMin, sMax, found);
    }
    else {
        int stack[64];
        int top = 0;
        stack[top++] = 0;
        while (top > 0) {
            const Node& n = mNodes[stack[--top]];
            if (o[u] < n.lo[u] || o[u] > n.hi[u] || o[v] < n.lo[v] || o[v] > n.hi[v]
                || n.hi[axis] < sMin || n.lo[axis] > sMax) {
                continue;
            }
            if (n.prim >= 0 && n.count != 1) {
                // the sweep of the grown primitive over the times of the node
                Prim grown = mPrims[n.prim];
                if (grown.sphere) {
                    grown.radius += n.grow;
                }
                else {
                    grown.z0 -= n.grow;
                    grown.z1 += n.grow;
                    grown.r0 += n.grow;
                    grown.r1 += n.grow;
                }
                Interval iv;
                const bool hit = grown.sphere ? SphereLine(n.pose, grown, axis, o, iv)
                                              : FrustumLine(n.pose, grown, axis, o, iv);
                if (!hit || iv.hi.s < sMin || iv.lo.s > sMax) {
                    continue;
                }
                // nothing to add to what was found
                bool inside = false;
                for (const Interval& f : found) {
                    inside = inside || (iv.lo.s >= f.lo.s && iv.hi.s <= f.hi.s);
                }
                if (inside) {
                    continue;
                }
            }
            if (n.count >= 0) {
                for (int i = n.first; i < n.first + n.count; i++) {
                    LineEntry(i, axis, o, sMin, sMax, found);
                }
            }
            else {
                stack[top++] = n.first;
                stack[top++] = n.right;
            }
        }
    }
    if (found.empty()) {
        return 0;
    }
    if (found.size() == 1) {
        out.push_back({found[0].lo, found[0].hi});
        return 1;
    }
    std::sort(found.begin(), found.end(), [](const Interval& a, const Interval& b) {
        return a.lo.s < b.lo.s;
    });
    // union
    const size_t first = out.size();
    Removal cur {found[0].lo, found[0].hi};
    for (size_t i = 1; i < found.size(); i++) {
        if (found[i].lo.s <= cur.hi.s) {
            if (found[i].hi.s > cur.hi.s) {
                cur.hi = found[i].hi;
            }
        }
        else {
            out.push_back(cur);
            cur = {found[i].lo, found[i].hi};
        }
    }
    out.push_back(cur);
    return (int)(out.size() - first);
}

bool Sweep::Normal(int axis, const double origin[3], const RemovalEnd& end, float n[3]) const
{
    const Tool& tool = *mMotion.tool;
    const Piece& p = mPieces[end.piece];
    const Prim& prim = mPrims[end.primitive];
    double x[3] = {origin[0], origin[1], origin[2]};
    x[axis] = end.s;
    double out[3];
    if (prim.sphere && mMotion.arc) {
        // the center of the sphere nearest to the point
        double t, tip[3], axisDir[3];
        ArcDistance(mMotion, x[0], x[1], t);
        PoseAt(t, tip, axisDir);
        for (int k = 0; k < 3; k++) {
            out[k] = tip[k] + prim.zc * p.a[k] - x[k];
        }
    }
    else if (prim.sphere) {
        double w[3];
        for (int k = 0; k < 3; k++) {
            w[k] = x[k] - (p.p0[k] + prim.zc * p.a[k]);
        }
        const double dd = Dot(p.d, p.d);
        const double tau = dd > 1e-24 ? std::clamp(Dot(w, p.d) / dd, 0.0, 1.0) : 0;
        for (int k = 0; k < 3; k++) {
            out[k] = -(w[k] - tau * p.d[k]);
        }
    }
    else {
        const Tool::Frustum& f = tool.Frustums()[prim.index];
        double w[3];
        if (mMotion.arc) {
            double tip[3], axisDir[3], t = end.t;
            if (t < 0) {
                ArcDistance(mMotion, x[0], x[1], t);
            }
            PoseAt(t, tip, axisDir);
            for (int k = 0; k < 3; k++) {
                w[k] = x[k] - tip[k];
            }
        }
        else {
            for (int k = 0; k < 3; k++) {
                w[k] = x[k] - p.p0[k] - end.t * p.d[k];
            }
        }
        const double lx = Dot(w, p.e1), ly = Dot(w, p.e2), lz = Dot(w, p.a);
        double nr, nz;
        switch (end.kind) {
            case CapBottom:
                nr = 0;
                nz = -1;
                break;
            case CapTop:
                nr = 0;
                nz = 1;
                break;
            case Rim0:
                if (f.sharp0) {
                    return false;
                }
                nr = f.n0r;
                nz = f.n0z;
                break;
            case Rim1: {
                if (prim.index1 < 0) {
                    return false;  // the top of a part of a split frustum
                }
                const Tool::Frustum& top = tool.Frustums()[prim.index1];
                if (prim.z1 < top.z1 || top.sharp1) {
                    return false;
                }
                nr = top.n1r;
                nz = top.n1z;
                break;
            }
            default: {
                // merged frustums: the normal of the edge
                const double u = std::clamp((lz - f.z0) / (f.z1 - f.z0), 0.0, 1.0);
                if (lz > f.z1) {
                    nr = f.n1r;
                    nz = f.n1z;
                }
                else {
                    nr = f.n0r + u * (f.n1r - f.n0r);
                    nz = f.n0z + u * (f.n1z - f.n0z);
                }
                break;
            }
        }
        const double rho = std::sqrt(lx * lx + ly * ly);
        if (rho <= 1e-12 && std::abs(nz) < 0.99) {
            return false;  // on the axis, the radial direction is unknown
        }
        const double rx = rho > 1e-12 ? lx / rho : 0, ry = rho > 1e-12 ? ly / rho : 0;
        // the material normal points into the tool
        for (int k = 0; k < 3; k++) {
            out[k] = -(nr * (rx * p.e1[k] + ry * p.e2[k]) + nz * p.a[k]);
        }
    }
    const double len = std::sqrt(Dot(out, out));
    if (!(len > 1e-12)) {
        return false;
    }
    for (int k = 0; k < 3; k++) {
        n[k] = (float)(out[k] / len);
    }
    return true;
}

}  // namespace CAMSimulator::Material
