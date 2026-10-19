// SPDX-License-Identifier: LGPL-2.1-or-later AND MIT

/***************************************************************************
 *   Copyright (c) 2024 Shai Seger <shaise at gmail>                       *
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
 *                                                                         *
 *   Portions of this code are taken from:                                 *
 *   "OpenGL 4 Shading Language cookbook" Third edition                    *
 *   Written by: David Wolff                                               *
 *   Published by: <packt> www.packt.com                                   *
 *   License: MIT License                                                  *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include "SimDisplay.h"
#include "MaterialTool.h"

#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <QOpenGLContext>
#include <algorithm>
#include <cmath>
#include <numbers>

// include this last as the defines can mess up other includes
#include "OpenGlWrapper.h"

namespace CAMSimulator
{

constexpr auto pi = std::numbers::pi_v<float>;

// limits for the GPU memory used by simulation checkpoints
constexpr int SIM_CHECKPOINT_MAX = 16;
constexpr std::size_t SIM_CHECKPOINT_MEMORY_BUDGET = 512 * 1024 * 1024;

void SimDisplay::InitShaders()
{
    // use shaders
    //   standard diffuse shader
    shader3D.CompileShader("StdDiffuse", VertShader3DNorm, FragShaderNorm);
    shader3D.UpdateEnvColor(lightPos, lightColor, ambientCol, 0.0f);

    //   invarted normal diffuse shader for inner mesh
    shaderInv3D.CompileShader("InvertNormal", VertShader3DInvNorm, FragShaderNorm);
    shaderInv3D.UpdateEnvColor(lightPos, lightColor, ambientCol, 0.0f);

    //   null shader to calculate meshes only (simulation stage)
    shaderFlat.CompileShader("Null", VertShader3DNorm, FragShaderFlat);

    //   texture shader to render Simulator FBO
    shaderSimFbo.CompileShader("Texture", VertShader2DFbo, FragShader2dFbo);
    shaderSimFbo.UpdateTextureSlot(0);

    // geometric shader - generate texture with all geometric info for further processing
    shaderGeom.CompileShader("Geometric", VertShaderGeom, FragShaderGeom);
    shaderGeomCloser.CompileShader("GeomCloser", VertShaderGeom, FragShaderGeom);
    shaderStock.CompileShader("StockGeometric", VertShaderStock, FragShaderStock);

    // SSAO shader - generate SSAO info and embed in texture buffer
    shaderSSAO.CompileShader("SSAO", VertShader2DFbo, FragShaderSSAO);
    shaderSSAO.UpdateRandomTexSlot(0);
    shaderSSAO.UpdatePositionTexSlot(1);
    shaderSSAO.UpdateNormalTexSlot(2);

    // SSAO blur shader - smooth generated SSAO texture
    shaderSSAOBlur.CompileShader("Blur", VertShader2DFbo, FragShaderSSAOBlur);
    shaderSSAOBlur.UpdateSsaoTexSlot(0);

    // SSAO lighting shader - apply lightig modified by SSAO calculations
    shaderSSAOLighting.CompileShader("SsaoLighting", VertShader2DFbo, FragShaderSSAOLighting);
    shaderSSAOLighting.UpdateColorTexSlot(0);
    shaderSSAOLighting.UpdatePositionTexSlot(1);
    shaderSSAOLighting.UpdateNormalTexSlot(2);
    shaderSSAOLighting.UpdateSsaoTexSlot(3);
    shaderSSAOLighting.UpdateEnvColor(lightPos, lightColor, ambientCol, 0.01f);

    // Mill Path Line Shader
    shaderLinePath.CompileShader("PathLine", VertShader3DLine, FragShader3DLine);

    // clear geometric info texture of fragments that passes the stencil test
    shaderClearGeom.CompileShader("ClearGeom", VertShader2DFbo, FragShaderClearGeom);
}

void SimDisplay::CreateFboQuad()
{
    float quadVertices[] = {// a quad that fills the entire screen in Normalized Device Coordinates.
                            // positions   // texCoords
                            -1.0f, 1.0f,  0.0f, 1.0f, -1.0f, -1.0f, 0.0f, 0.0f,
                            1.0f,  -1.0f, 1.0f, 0.0f, -1.0f, 1.0f,  0.0f, 1.0f,
                            1.0f,  -1.0f, 1.0f, 0.0f, 1.0f,  1.0f,  1.0f, 1.0f
    };

    glGenBuffers(1, &mFboQuadVBO);
    glBindBuffer(GL_ARRAY_BUFFER, mFboQuadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices[0], GL_STATIC_DRAW);
}

void SimDisplay::SetupVertexAttribs() const
{
    glBindBuffer(GL_ARRAY_BUFFER, mFboQuadVBO);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
}

void SimDisplay::CreateGBufTex(
    GLenum texUnit,
    GLint intFormat,
    GLenum format,
    GLenum type,
    GLuint& texid,
    int width,
    int height
)
{
    glActiveTexture(texUnit);
    glGenTextures(1, &texid);
    glBindTexture(GL_TEXTURE_2D, texid);
    width = width < 0 ? mWidth : width;
    height = height < 0 ? mHeight : height;
    glTexImage2D(GL_TEXTURE_2D, 0, intFormat, width, height, 0, format, type, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
}

void SimDisplay::UniformHemisphere(vec3& randVec)
{
    float x1 = distr01(generator);
    float x2 = distr01(generator);
    float s = sqrt(1.0f - x1 * x1);
    randVec[0] = cosf(pi * 2 * x2) * s;
    randVec[1] = sinf(pi * 2 * x2) * s;
    randVec[2] = x1;
}

void SimDisplay::UniformCircle(vec3& randVec)
{
    float x = distr01(generator);
    randVec[0] = cosf(pi * 2 * x);
    randVec[1] = sinf(pi * 2 * x);
    randVec[2] = 0;
}

bool SimDisplay::CreateGBufferFbo(
    unsigned int& fbo,
    unsigned int& colTex,
    unsigned int& posTex,
    unsigned int& normTex,
    unsigned int& rboDepthStencil
)
{
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    // a color texture for the frame buffer
    CreateGBufTex(GL_TEXTURE0, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, colTex);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colTex, 0);

    // a position texture for the frame buffer
    CreateGBufTex(GL_TEXTURE1, GL_RGB32F, GL_RGBA, GL_FLOAT, posTex);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, posTex, 0);

    // a normal texture for the frame buffer
    CreateGBufTex(GL_TEXTURE2, GL_RGB32F, GL_RGBA, GL_FLOAT, normTex);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, normTex, 0);

    unsigned int attachments[3] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
    glDrawBuffers(3, attachments);

    glGenRenderbuffers(1, &rboDepthStencil);
    glBindRenderbuffer(GL_RENDERBUFFER, rboDepthStencil);
    glRenderbufferStorage(
        GL_RENDERBUFFER,
        GL_DEPTH24_STENCIL8,
        mWidth,
        mHeight
    );  // use a single renderbuffer object for both a depth AND stencil buffer.
    glFramebufferRenderbuffer(
        GL_FRAMEBUFFER,
        GL_DEPTH_STENCIL_ATTACHMENT,
        GL_RENDERBUFFER,
        rboDepthStencil
    );  // now actually attach it

    return glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
}

void SimDisplay::CreateDisplayFbos()
{
    simCacheValid = false;

    // setup frame buffer for simulation
    bool complete
        = CreateGBufferFbo(mFbo, mFboColTexture, mFboPosTexture, mFboNormTexture, mRboDepthStencil);
    mTargetFbo = mFbo;
    if (!complete) {
        return;
    }

    // transparent objects
    if (
        !CreateGBufferFbo(mTransFbo, mTransColTexture, mTransPosTexture, mTransNormTexture, mTransRboDepthStencil)
    ) {
        GLDELETE_FRAMEBUFFER(mTransFbo);
    }

    // setup frame buffer that keeps the simulation result between frames
    if (
        mSimFboSupported
        && !CreateGBufferFbo(mSimFbo, mSimColTexture, mSimPosTexture, mSimNormTexture, mSimRboDepthStencil)
    ) {
        GLDELETE_FRAMEBUFFER(mSimFbo);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void SimDisplay::CreateSsaoFbos()
{
    mSsaoValid = true;

    // setup framebuffer for SSAO processing
    glGenFramebuffers(1, &mSsaoFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, mSsaoFbo);
    // SSAO color buffer
    // ambient occlusion is smooth, computing it at half resolution is enough
    mSsaoWidth = std::max(1, mWidth / 2);
    mSsaoHeight = std::max(1, mHeight / 2);
    CreateGBufTex(GL_TEXTURE0, GL_R16F, GL_RED, GL_FLOAT, mFboSsaoTexture, mSsaoWidth, mSsaoHeight);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mFboSsaoTexture, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        mSsaoValid = false;
        return;
    }

    // setup framebuffer for SSAO blur processing
    glGenFramebuffers(1, &mSsaoBlurFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, mSsaoBlurFbo);
    CreateGBufTex(GL_TEXTURE0, GL_R16F, GL_RED, GL_FLOAT, mFboSsaoBlurTexture, mSsaoWidth, mSsaoHeight);
    // upscaled to full resolution in the lighting pass
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mFboSsaoBlurTexture, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        mSsaoValid = false;
        return;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // generate sample kernel
    int kernSize = 32;
    for (int i = 0; i < kernSize; i++) {
        vec3 sample;
        UniformHemisphere(sample);
        float scale = ((float)(i * i)) / (kernSize * kernSize);
        float interpScale = 0.1f * (1.0f - scale) + scale;
        vec3_scale(sample, sample, interpScale);
        mSsaoKernel.push_back(*(Point3D*)sample);
    }
    shaderSSAO.Activate();
    shaderSSAO.UpdateKernelVals(mSsaoKernel.size(), &mSsaoKernel[0].x);

    // generate random direction texture
    int randSize = 4 * 4;
    std::vector<Point3D> randDirections;
    for (int i = 0; i < randSize; i++) {
        vec3 randvec;
        UniformCircle(randvec);
        randDirections.push_back(*(Point3D*)randvec);
    }

    glGenTextures(1, &mFboRandTexture);
    glBindTexture(GL_TEXTURE_2D, mFboRandTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, 4, 4, 0, GL_RGB, GL_FLOAT, &randDirections[0].x);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
}

SimDisplay::~SimDisplay()
{
    CleanGL();
}

void SimDisplay::InitGL()
{
    if (displayInitiated) {
        return;
    }

    // setup light object
    mlightObject.GenerateBoxStock(-0.5f, -0.5f, -0.5f, 1, 1, 1);

    // keeping the simulation result in a separate FBO requires blitting between frame buffers
    // (OpenGL 3.0, OpenGL ES 3.0 or ARB_framebuffer_object)
    const QOpenGLContext* context = QOpenGLContext::currentContext();
    mSimFboSupported = context
        && (context->format().version() >= qMakePair(3, 0)
            || context->hasExtension("GL_ARB_framebuffer_object"));

    InitShaders();
    CreateFboQuad();

    displayInitiated = true;

    UpdateWindowScale(800, 600);
}

void SimDisplay::CleanFbos()
{
    simCacheValid = false;
    mTargetFbo = 0;

    // cleanup frame buffers
    GLDELETE_FRAMEBUFFER(mFbo);
    GLDELETE_FRAMEBUFFER(mTransFbo);
    GLDELETE_FRAMEBUFFER(mSimFbo);
    GLDELETE_FRAMEBUFFER(mSsaoFbo);
    GLDELETE_FRAMEBUFFER(mSsaoBlurFbo);

    // cleanup fbo textures
    GLDELETE_TEXTURE(mFboColTexture);
    GLDELETE_TEXTURE(mFboPosTexture);
    GLDELETE_TEXTURE(mFboNormTexture);
    GLDELETE_TEXTURE(mFboSsaoTexture);
    GLDELETE_TEXTURE(mFboSsaoBlurTexture);
    GLDELETE_TEXTURE(mFboRandTexture);
    GLDELETE_RENDERBUFFER(mRboDepthStencil);
    GLDELETE_TEXTURE(mTransColTexture);
    GLDELETE_TEXTURE(mTransPosTexture);
    GLDELETE_TEXTURE(mTransNormTexture);
    GLDELETE_RENDERBUFFER(mTransRboDepthStencil);
    mTransparentLayer = false;
    GLDELETE_TEXTURE(mSimColTexture);
    GLDELETE_TEXTURE(mSimPosTexture);
    GLDELETE_TEXTURE(mSimNormTexture);
    GLDELETE_RENDERBUFFER(mSimRboDepthStencil);

    for (auto& checkpoint : mSimCheckpoints) {
        DeleteGBufferFbo(checkpoint);
    }
    mSimCheckpoints.clear();
}

void SimDisplay::CleanGL()
{
    CleanFbos();

    // cleanup geometry
    GLDELETE_BUFFER(mFboQuadVBO);

    GLDELETE_TEXTURE(mOperationColorTexture);
    mOperationColors.clear();

    // cleanup shaders
    shader3D.Destroy();
    shaderInv3D.Destroy();
    shaderFlat.Destroy();
    shaderSimFbo.Destroy();
    shaderGeom.Destroy();
    shaderStock.Destroy();
    shaderSSAO.Destroy();
    shaderSSAOLighting.Destroy();
    shaderSSAOBlur.Destroy();
    shaderClearGeom.Destroy();

    displayInitiated = false;
}

void SimDisplay::PrepareFrameBuffer()
{
    mTargetFbo = mFbo;
    glBindFramebuffer(GL_FRAMEBUFFER, mFbo);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
}

bool SimDisplay::CanCacheSimulation() const
{
    return mSimFbo != 0;
}

// Direct the following simulation passes to the simulation FBO. When 'clear' is false, the
// passes are added on top of the previous simulation result.
void SimDisplay::StartSimulationPass(bool clear)
{
    mTargetFbo = mSimFbo;
    glBindFramebuffer(GL_FRAMEBUFFER, mSimFbo);
    if (clear) {
        glClearColor(0, 0, 0, 0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    }
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
}

// Copy the simulation result to the display FBO, so tool, path and base shape can be added
// on top of it without changing the simulation FBO.
void SimDisplay::EndSimulationPass()
{
    mTargetFbo = mFbo;
    BlitGBuffer(mSimFbo, mFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, mFbo);
    simCacheValid = true;
}

// Copy all attachments of one geometric info frame buffer to another
void SimDisplay::BlitGBuffer(unsigned int fromFbo, unsigned int toFbo)
{
    // make sure all buffers are writable
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);
    glStencilMask(0xFF);

    glBindFramebuffer(GL_READ_FRAMEBUFFER, fromFbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, toFbo);

    // blit only copies a single color buffer at a time
    for (int i = 0; i < 3; i++) {
        unsigned int buffers[3] = {GL_NONE, GL_NONE, GL_NONE};
        buffers[i] = GL_COLOR_ATTACHMENT0 + i;
        glReadBuffer(buffers[i]);
        glDrawBuffers(i + 1, buffers);
        glBlitFramebuffer(0, 0, mWidth, mHeight, 0, 0, mWidth, mHeight, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    }
    glBlitFramebuffer(
        0,
        0,
        mWidth,
        mHeight,
        0,
        0,
        mWidth,
        mHeight,
        GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT,
        GL_NEAREST
    );

    unsigned int attachments[3] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
    glDrawBuffers(3, attachments);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
}

// Number of simulation checkpoints that fit the memory budget at the current window size
int SimDisplay::MaxSimulationCheckpoints() const
{
    if (!CanCacheSimulation()) {
        return 0;
    }
    // color (RGBA8) + position and normal (RGB32F, usually padded to RGBA) + depth/stencil
    constexpr std::size_t bytesPerPixel = 4 + 16 + 16 + 4;
    const std::size_t bytesPerCheckpoint = (std::size_t)mWidth * mHeight * bytesPerPixel;
    const std::size_t count = SIM_CHECKPOINT_MEMORY_BUDGET
        / std::max<std::size_t>(bytesPerCheckpoint, 1);
    return (int)std::min<std::size_t>(count, SIM_CHECKPOINT_MAX);
}

// Copy the simulation frame buffer to checkpoint 'slot'. Returns false if it could not be created.
bool SimDisplay::SaveSimulationCheckpoint(int slot)
{
    if (slot < 0 || slot >= MaxSimulationCheckpoints()) {
        return false;
    }
    if (slot >= (int)mSimCheckpoints.size()) {
        mSimCheckpoints.resize(slot + 1);
    }

    GBufferFbo& checkpoint = mSimCheckpoints[slot];
    if (checkpoint.fbo == 0) {
        const bool complete = CreateGBufferFbo(
            checkpoint.fbo,
            checkpoint.colTexture,
            checkpoint.posTexture,
            checkpoint.normTexture,
            checkpoint.rboDepthStencil
        );
        if (!complete) {
            DeleteGBufferFbo(checkpoint);
            return false;
        }
    }

    BlitGBuffer(mSimFbo, checkpoint.fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, mTargetFbo);
    return true;
}

// Copy checkpoint 'slot' to the simulation frame buffer
void SimDisplay::RestoreSimulationCheckpoint(int slot)
{
    BlitGBuffer(mSimCheckpoints.at(slot).fbo, mSimFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, mTargetFbo);
}

void SimDisplay::DeleteGBufferFbo(GBufferFbo& gbuf)
{
    GLDELETE_FRAMEBUFFER(gbuf.fbo);
    GLDELETE_TEXTURE(gbuf.colTexture);
    GLDELETE_TEXTURE(gbuf.posTexture);
    GLDELETE_TEXTURE(gbuf.normTexture);
    GLDELETE_RENDERBUFFER(gbuf.rboDepthStencil);
}

// Reset the geometric info of all pixels where the stencil value is 0 to the cleared state
void SimDisplay::ClearGeometryOutsideStencil()
{
    glBindFramebuffer(GL_FRAMEBUFFER, mTargetFbo);
    shaderClearGeom.Activate();
    SetupVertexAttribs();
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_EQUAL, 0, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
}

void SimDisplay::StartDepthPass()
{
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    shaderFlat.Activate();
    shaderFlat.UpdateViewMat(mMatLookAt);
}

void SimDisplay::StartGeometryPass(const vec3& objColor, bool invertNormals)
{
    glBindFramebuffer(GL_FRAMEBUFFER, mTargetFbo);
    shaderGeom.Activate();
    shaderGeom.UpdateNormalState(invertNormals);
    shaderGeom.UpdateViewMat(mMatLookAt);
    shaderGeom.UpdateObjColor(objColor);
    glUniform1f(glGetUniformLocation(shaderGeom.shaderId, "opacity"), mOpacity);
    glEnable(GL_CULL_FACE);
    glDisable(GL_BLEND);
}

void SimDisplay::StartStockGeometryPass(const vec3& objColor, const StockColoring& coloring)
{
    glBindFramebuffer(GL_FRAMEBUFFER, mTargetFbo);
    shaderStock.Activate();
    shaderStock.UpdateViewMat(mMatLookAt);
    shaderStock.UpdateObjColor(objColor);
    const unsigned int id = shaderStock.shaderId;
    const int colors = std::min((int)coloring.operationColors.size() / 3, Material::MaxTag);
    glUniform1i(
        glGetUniformLocation(id, "colorByOperation"),
        coloring.byOperation && colors > 0 ? 1 : 0
    );
    // the colors in a texture, read by the vertex shader
    glActiveTexture(GL_TEXTURE5);
    if (colors > 0 && (mOperationColorTexture == 0 || mOperationColors != coloring.operationColors)) {
        mOperationColors = coloring.operationColors;
        if (mOperationColorTexture == 0) {
            glGenTextures(1, &mOperationColorTexture);
        }
        glBindTexture(GL_TEXTURE_2D, mOperationColorTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, colors, 1, 0, GL_RGB, GL_FLOAT, mOperationColors.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    glBindTexture(GL_TEXTURE_2D, mOperationColorTexture);
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(glGetUniformLocation(id, "operationColors"), 5);
    glUniform1f(glGetUniformLocation(id, "operationCount"), (float)std::max(colors, 1));
    glUniform1f(glGetUniformLocation(id, "thinThreshold"), coloring.thinThreshold);
    glUniform1f(glGetUniformLocation(id, "opacity"), mOpacity);
    glUniform3fv(glGetUniformLocation(id, "thinColor"), 1, coloring.thinColor);
    glEnable(GL_CULL_FACE);
    glDisable(GL_BLEND);
}

// A 'closer' geometry pass is similar to std geometry pass, but render the objects
// slightly closer to the camera. This mitigates overlapping faces artifacts.
void SimDisplay::StartCloserGeometryPass(const vec3& objColor)
{
    glBindFramebuffer(GL_FRAMEBUFFER, mTargetFbo);
    shaderGeomCloser.Activate();
    shaderGeomCloser.UpdateNormalState(false);
    shaderGeomCloser.UpdateViewMat(mMatLookAt);
    shaderGeomCloser.UpdateObjColor(objColor);
    glUniform1f(glGetUniformLocation(shaderGeomCloser.shaderId, "opacity"), mOpacity);
    glEnable(GL_CULL_FACE);
    glDisable(GL_BLEND);
}

bool SimDisplay::StartTransparentLayer()
{
    if (mTransFbo == 0 || mFbo == 0) {
        mTransparentLayer = false;
        return false;
    }
    // the depth of the opaque objects: transparent surfaces behind them are hidden
    glBindFramebuffer(GL_READ_FRAMEBUFFER, mFbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, mTransFbo);
    glBlitFramebuffer(0, 0, mWidth, mHeight, 0, 0, mWidth, mHeight, GL_DEPTH_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, mTransFbo);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    mTargetFbo = mTransFbo;
    mTransparentLayer = true;
    return true;
}

void SimDisplay::EndTransparentLayer()
{
    mTargetFbo = mFbo;
    mOpacity = 1;
}

void SimDisplay::ComposeTransparentLayer()
{
    if (!mTransparentLayer) {
        return;
    }
    // lit like the opaque objects, without ambient occlusion, blended by the opacity in the alpha
    // of the color
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    shaderSSAOLighting.Activate();
    shaderSSAOLighting.UpdateColorTexSlot(0);
    shaderSSAOLighting.UpdatePositionTexSlot(1);
    shaderSSAOLighting.UpdateNormalTexSlot(2);
    shaderSSAOLighting.UpdateSsaoActive(false);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mTransColTexture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, mTransPosTexture);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, mTransNormTexture);
    glEnable(GL_BLEND);
    // the alpha of the view stays opaque, it is composited with the window
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ZERO, GL_ONE);
    SetupVertexAttribs();
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

void SimDisplay::RenderLightObject()
{
    shaderFlat.Activate();
    shaderFlat.UpdateObjColor(lightColor);
    mlightObject.render();
}

void SimDisplay::ScaleViewToStock(StockObject* obj)
{
    mMaxStockDimension = std::max(std::max(obj->size[0], obj->size[1]), obj->size[2]);
    UpdateProjectionMatrix();
}

void SimDisplay::RenderResult(bool recalculate, bool ssao)
{
    if (!displayInitiated) {
        return;
    }

    if (mSsaoValid && ssao) {
        RenderResultSSAO(recalculate);
    }
    else {
        RenderResultStandard();
    }
    ComposeTransparentLayer();
}

void SimDisplay::RenderResultStandard()
{
    // set default frame buffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // display the sim result within the FBO
    shaderSSAOLighting.Activate();
    shaderSSAOLighting.UpdateColorTexSlot(0);
    shaderSSAOLighting.UpdatePositionTexSlot(1);
    shaderSSAOLighting.UpdateNormalTexSlot(2);
    shaderSSAOLighting.UpdateSsaoActive(false);
    // shaderSimFbo.Activate();
    SetupVertexAttribs();
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mFboColTexture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, mFboPosTexture);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, mFboNormTexture);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

void SimDisplay::RenderResultSSAO(bool recalculate)
{
    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    if (recalculate) {
        // generate SSAO texture
        glBindFramebuffer(GL_FRAMEBUFFER, mSsaoFbo);
        shaderSSAO.Activate();
        shaderSSAO.UpdateRandomTexSlot(0);
        shaderSSAO.UpdatePositionTexSlot(1);
        shaderSSAO.UpdateNormalTexSlot(2);
        glViewport(0, 0, mSsaoWidth, mSsaoHeight);
        shaderSSAO.UpdateScreenDimension(mSsaoWidth, mSsaoHeight);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, mFboRandTexture);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, mFboPosTexture);
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, mFboNormTexture);
        SetupVertexAttribs();
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // blur SSAO texture to remove noise
        glBindFramebuffer(GL_FRAMEBUFFER, mSsaoBlurFbo);
        glClear(GL_COLOR_BUFFER_BIT);
        shaderSSAOBlur.Activate();
        shaderSSAOBlur.UpdateSsaoTexSlot(0);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, mFboSsaoTexture);
        shaderSSAOBlur.UpdateScreenDimension(mSsaoWidth, mSsaoHeight);
        SetupVertexAttribs();
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glViewport(0, 0, mWidth, mHeight);
    }

    // lighting pass:
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    shaderSSAOLighting.Activate();
    shaderSSAOLighting.UpdateColorTexSlot(0);
    shaderSSAOLighting.UpdatePositionTexSlot(1);
    shaderSSAOLighting.UpdateNormalTexSlot(2);
    shaderSSAOLighting.UpdateSsaoTexSlot(3);
    shaderSSAOLighting.UpdateSsaoActive(true);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mFboColTexture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, mFboPosTexture);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, mFboNormTexture);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, mFboSsaoBlurTexture);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    SetupVertexAttribs();
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

void SimDisplay::SetPathColor(const vec3& normal, const vec3& rapid)
{
    pathLineColor[0] = normal[0];
    pathLineColor[1] = normal[1];
    pathLineColor[2] = normal[2];

    // TODO: Different color for rapid moves is not supported for now.

    (void)rapid;
}

void SimDisplay::SetupLinePathPass(int curSegment, bool isHidden)
{
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDepthFunc(isHidden ? GL_GREATER : GL_LESS);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineWidth(2);
    shaderLinePath.Activate();
    pathLineColor[3] = isHidden ? 0.1f : 1.0f;
    shaderLinePath.UpdateObjColorAlpha(pathLineColor);
    shaderLinePath.UpdateObjColor(pathLineColorPassed);
    shaderLinePath.UpdateCurSegment(curSegment);
    shaderLinePath.UpdateViewMat(mMatLookAt);
}

void SimDisplay::UpdateWindowScale(int width, int height)
{
    if (!displayInitiated || (width == mWidth && height == mHeight)) {
        return;
    }

    mWidth = width;
    mHeight = height;

    if (mFbo != 0) {
        glBindFramebuffer(GL_FRAMEBUFFER, mFbo);
        CleanFbos();
    }

    CreateDisplayFbos();
    CreateSsaoFbos();
    UpdateProjectionMatrix();
}

void SimDisplay::UpdateCamera(const SoCamera& camera)
{
    if (!displayInitiated) {
        return;
    }

    UpdateCameraView(camera);
    UpdateCameraProjection(camera);
}

void SimDisplay::UpdateCameraView(const SoCamera& camera)
{

    const SbVec3f position = camera.position.getValue();
    const SbRotation orientation = camera.orientation.getValue();

    if (position == mCameraPosition && orientation == mCameraOrientation) {
        return;
    }

    mCameraPosition = position;
    mCameraOrientation = orientation;

    UpdateViewMatrix();
}

void SimDisplay::UpdateCameraProjection(const SoCamera& camera)
{
    float heightAngle = std::numbers::pi / 4;
    float height = 100.0f;

    const auto perspective = dynamic_cast<const SoPerspectiveCamera*>(&camera);
    const auto orthographic = dynamic_cast<const SoOrthographicCamera*>(&camera);

    // TODO: We can't use the values from the camera here because the dummy viewer never actually
    // renders the scene and therefore the nearDistance and farDistance of the camera are never
    // updated. Figure out a way to update those values without rendering the scene.

#if 0

    const float nearDistance = camera.nearDistance.getValue();
    const float farDistance = camera.farDistance.getValue();

#else

    float nearDistance = 0.0;
    float farDistance = 0.0;

#endif

    if (perspective) {
        heightAngle = perspective->heightAngle.getValue();

        nearDistance = mMaxStockDimension * 0.001f;
        farDistance = mMaxStockDimension * 10.0f;
    }
    else if (orthographic) {
        height = orthographic->height.getValue();

        nearDistance = -mMaxStockDimension * 10.0f;
        farDistance = mMaxStockDimension * 10.0f;
    }

    if ((bool)perspective == mCameraPerspective && heightAngle == mCameraHeightAngle
        && height == mCameraHeight && nearDistance == mCameraNearDistance
        && farDistance == mCameraFarDistance) {
        return;
    }

    mCameraPerspective = (bool)perspective;
    mCameraHeightAngle = heightAngle;
    mCameraHeight = height;
    mCameraNearDistance = nearDistance;
    mCameraFarDistance = farDistance;

    UpdateProjectionMatrix();
}

void SimDisplay::UpdateViewMatrix()
{
    SbVec3f up(0, 1, 0);
    mCameraOrientation.multVec(up, up);

    SbVec3f dir(0, 0, -1);
    mCameraOrientation.multVec(dir, dir);

    const auto target = mCameraPosition + dir;
    mat4x4_look_at(mMatLookAt, mCameraPosition.getValue(), target.getValue(), up.getValue());

    simCacheValid = false;
    updateDisplay = true;
}

void SimDisplay::UpdateProjectionMatrix()
{
    // Setup projection

    const float aspect = (float)mWidth / mHeight;

    mat4x4 projmat;

    if (mCameraPerspective) {
        mat4x4_perspective(projmat, mCameraHeightAngle, aspect, mCameraNearDistance, mCameraFarDistance);
    }
    else {
        const float h = mCameraHeight;
        const float w = mCameraHeight * aspect;
        mat4x4_ortho(projmat, -w / 2, w / 2, -h / 2, h / 2, mCameraNearDistance, mCameraFarDistance);
    }

    shader3D.Activate();
    shader3D.UpdateProjectionMat(projmat);
    shaderInv3D.Activate();
    shaderInv3D.UpdateProjectionMat(projmat);
    shaderFlat.Activate();
    shaderFlat.UpdateProjectionMat(projmat);
    shaderGeom.Activate();
    shaderGeom.UpdateProjectionMat(projmat);
    shaderStock.Activate();
    shaderStock.UpdateProjectionMat(projmat);
    shaderSSAO.Activate();
    shaderSSAO.UpdateProjectionMat(projmat);
    shaderLinePath.Activate();
    shaderLinePath.UpdateProjectionMat(projmat);
    // a few pixels, more than the distance of the mesh from the exact surface
    constexpr float BiasPixels = 3;
    if (mCameraPerspective) {
        shaderLinePath.UpdateDepthBias(BiasPixels * 2 * std::tan(mCameraHeightAngle / 2) / mHeight, 0);
    }
    else {
        shaderLinePath.UpdateDepthBias(0, BiasPixels * mCameraHeight / mHeight);
    }

    projmat[2][2] *= 0.99999F;
    shaderGeomCloser.Activate();
    shaderGeomCloser.UpdateProjectionMat(projmat);

    simCacheValid = false;
    updateDisplay = true;
}

}  // namespace CAMSimulator
