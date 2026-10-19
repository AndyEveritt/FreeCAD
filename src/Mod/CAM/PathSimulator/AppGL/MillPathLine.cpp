// SPDX-License-Identifier: LGPL-2.1-or-later

#include "MillPathLine.h"

#include "Shader.h"

// include this last as the defines can mess up other includes
#include "OpenGlWrapper.h"

namespace CAMSimulator
{

void MillPathLine::GenerateModel()
{
    // separate lines rather than a strip, so every line has the attributes of its segment
    std::vector<MillPathPosition> lines;
    lines.reserve(2 * MillPathPointsBuffer.size());
    for (size_t i = 1; i < MillPathPointsBuffer.size(); i++) {
        const MillPathPosition& end = MillPathPointsBuffer[i];
        MillPathPosition start = MillPathPointsBuffer[i - 1];
        start.SegmentId = end.SegmentId;
        start.Rapid = end.Rapid;
        lines.push_back(start);
        lines.push_back(end);
    }
    mNumVerts = (int)lines.size();
    void* vbuffer = lines.data();

    // vertex buffer
    glGenBuffers(1, &mVbo);
    glBindBuffer(GL_ARRAY_BUFFER, mVbo);
    glBufferData(GL_ARRAY_BUFFER, mNumVerts * sizeof(MillPathPosition), vbuffer, GL_STATIC_DRAW);

    // free
    MillPathPointsBuffer.clear();
}

void MillPathLine::SetupVertexAttibs()
{
    glBindBuffer(GL_ARRAY_BUFFER, mVbo);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(MillPathPosition),
        (void*)offsetof(MillPathPosition, X)
    );
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(
        1,
        1,
        GL_INT,
        GL_FALSE,
        sizeof(MillPathPosition),
        (void*)offsetof(MillPathPosition, SegmentId)
    );
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(
        2,
        1,
        GL_FLOAT,
        GL_FALSE,
        sizeof(MillPathPosition),
        (void*)offsetof(MillPathPosition, Rapid)
    );
}

void MillPathLine::Clear()
{
    MillPathPointsBuffer.clear();
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    GLDELETE_BUFFER(mVbo);
}

void MillPathLine::Render()
{
    SetupVertexAttibs();
    glDrawArrays(GL_LINES, 0, mNumVerts);
    glDisableVertexAttribArray(2);
}

}  // namespace CAMSimulator
