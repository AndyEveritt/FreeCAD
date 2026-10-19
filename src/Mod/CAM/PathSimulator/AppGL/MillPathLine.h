// SPDX-License-Identifier: LGPL-2.1-or-later

#pragma once

#include <vector>

namespace CAMSimulator
{

struct MillPathPosition
{
    float X, Y, Z;
    int SegmentId;
    float Rapid = 0;  // 1 on a rapid move (G0)
};

class MillPathLine
{
public:
    // The lines between the points of MillPathPointsBuffer, each with the segment and the rapid
    // flag of its end
    void GenerateModel();
    void SetupVertexAttibs();
    void Clear();
    void Render();

public:
    std::vector<MillPathPosition> MillPathPointsBuffer;

protected:
    unsigned int mVbo = 0;
    int mNumVerts;
};

}  // namespace CAMSimulator
