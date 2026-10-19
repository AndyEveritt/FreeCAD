// SPDX-License-Identifier: LGPL-2.1-or-later

// Tools and stocks of the tests of the material simulation

#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace MaterialTest
{

inline constexpr float ToolLength = 40;

inline std::vector<float> FlatProfile(float radius)
{
    return {radius, ToolLength, radius, 0, 0, 0};
}

// a quarter circle in 'segments' edges, from (cr + radius, cz) down to (cr, cz - radius)
inline void Arc(std::vector<float>& profile, float cr, float cz, float radius, int segments)
{
    for (int i = 1; i <= segments; i++) {
        const float a = i * (float)M_PI / 2 / segments;
        profile.push_back(cr + radius * std::cos(a));
        profile.push_back(cz - radius * std::sin(a));
    }
}

inline std::vector<float> BallProfile(float radius, int segments = 32)
{
    std::vector<float> profile = {radius, ToolLength, radius, radius};
    Arc(profile, 0, radius, radius, segments);
    return profile;
}

inline std::vector<float> BullNoseProfile(float radius, float corner)
{
    std::vector<float> profile = {radius, ToolLength, radius, corner};
    Arc(profile, radius - corner, corner, corner, 16);
    profile.push_back(0);
    profile.push_back(0);
    return profile;
}

inline std::vector<float> VBitProfile(float radius, float halfAngleDeg)
{
    const float h = radius / std::tan(halfAngleDeg * (float)M_PI / 180);
    return {radius, ToolLength, radius, h, 0, 0};
}

// neck radius 3 above height 6, widening to radius 7 at the tip
inline std::vector<float> DovetailProfile()
{
    return {3, ToolLength, 3, 6, 7, 0, 0, 0};
}

// a disk of radius 8 and height 3 on a neck of radius 2.5, with a stub of radius 2.5 below
inline std::vector<float> TSlotProfile()
{
    return {2.5f, ToolLength, 2.5f, 3, 8, 3, 8, 0, 2.5f, 0, 2.5f, -1};
}

// a wide head and a wide tip on a thin neck: lines along the axis meet it twice
inline std::vector<float> WaistedProfile()
{
    return {2, ToolLength, 2, 6, 0.5f, 6, 0.5f, 2, 2, 2, 2, 0, 0, 0};
}

// a probe is modeled below its tip
inline std::vector<float> ProbeProfile()
{
    return {1, 0, 1, -20, 0, -21};
}

// box stock from lo to hi
inline void MakeBox(
    const double lo[3],
    const double hi[3],
    std::vector<float>& xyz,
    std::vector<uint32_t>& triangles
)
{
    xyz.clear();
    for (int i = 0; i < 8; i++) {
        xyz.push_back((float)(i & 1 ? hi[0] : lo[0]));
        xyz.push_back((float)(i & 2 ? hi[1] : lo[1]));
        xyz.push_back((float)(i & 4 ? hi[2] : lo[2]));
    }
    triangles = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4,
                 2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
}

}  // namespace MaterialTest
