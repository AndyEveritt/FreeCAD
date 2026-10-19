// SPDX-License-Identifier: LGPL-2.1-or-later

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
 ***************************************************************************/

#pragma once

#include "Shader.h"
#include "StockObject.h"
#include <Inventor/SbRotation.h>
#include <Inventor/SbVec3f.h>
#include <QOpenGLFunctions>
#include <numbers>
#include <random>
#include <vector>

class SoCamera;
class SoPerspectiveCamera;

namespace CAMSimulator
{

// Colors of the faces of the simulated stock, see SimDisplay::StartStockGeometryPass
struct StockColoring
{
    // cut faces in the color of the operation that cut them, by operation tag (see OperationTag)
    bool byOperation = false;
    std::vector<float> operationColors;  // rgb of the tags 1, 2, ...
    // faces in front of material thinner than this in the thin color, 0 for none
    float thinThreshold = 0;
    vec3 thinColor = {0.9f, 0.08f, 0.08f};
    // faces by their distance from the model (instead of the colors above): within the tolerance
    // green, material left blue, cut into the model red, darker up to 10 tolerances away
    bool deviation = false;
    float tolerance = 0.05f;
};

// The object at a position of the view, see SimDisplay::RequestPick
struct PickResult
{
    int x = -1, y = -1;
    bool pending = false;
    bool done = false;
    bool stockInLayer = false;  // found in a transparent layer
    bool stock = false;         // the stock is there, else nothing or another object
    int tag = 0;                // operation tag of its face (0 for uncut)
    float position[3] = {0, 0, 0};
};

struct Point3D
{
    float x, y, z;
};

// frame buffer holding the geometric info of a rendered simulation
struct GBufferFbo
{
    unsigned int fbo = 0;
    unsigned int colTexture = 0;
    unsigned int posTexture = 0;
    unsigned int normTexture = 0;
    unsigned int rboDepthStencil = 0;
};

class SimDisplay
{
public:
    ~SimDisplay();
    void InitGL();
    void CleanGL();
    void CleanFbos();
    void PrepareFrameBuffer();
    void StartDepthPass();
    void StartGeometryPass(const vec3& objColor, bool invertNormals);
    void StartCloserGeometryPass(const vec3& objColor);
    void StartStockGeometryPass(const vec3& objColor, const StockColoring& coloring);
    // Lines over the opaque objects, in a color: positions in attribute 0
    void StartOverlayPass(const vec3& color);

    // Transparent objects are rendered after the opaque ones, in layers (depth peeling): every
    // layer draws them between StartTransparentLayer and EndTransparentLayer, with the opacity of
    // SetOpacity, both sides of their faces, and keeps the surfaces behind those of the layer
    // before. The layers are lit and blended over the opaque objects from the back, so the
    // surfaces behind a transparent one show through it, like the model inside the stock and the
    // far side of a wall. StartTransparentLayer returns false beyond the last layer, or if not
    // supported.
    static constexpr int TransparentLayers = 4;
    bool StartTransparentLayer(int layer);
    void EndTransparentLayer();
    // No transparent objects in the next frames
    void ClearTransparentLayer()
    {
        mTransparentLayer = false;
        mLayerCount = 0;
    }
    void SetOpacity(float opacity)
    {
        mOpacity = opacity;
    }
    void StartSimulationPass(bool clear);
    void EndSimulationPass();
    void ClearGeometryOutsideStencil();
    bool CanCacheSimulation() const;
    int MaxSimulationCheckpoints() const;
    bool SaveSimulationCheckpoint(int slot);
    void RestoreSimulationCheckpoint(int slot);
    void RenderLightObject();
    void ScaleViewToStock(StockObject* obj);
    void RenderResult(bool recalculate, bool ssao);
    void RenderResultStandard();
    void RenderResultSSAO(bool recalculate);
    void SetOutlines(bool enabled);
    void SetupLinePathPass(int curSegment, bool isHidden);
    void UpdateWindowScale(int width, int height);
    void UpdateCamera(const SoCamera& camera);

    void SetPathColor(const vec3& normal, const vec3& rapid);

    // The object at a pixel of the view (from the bottom left) in the next frame, which is drawn
    // completely; TakePick gives it after that frame
    void RequestPick(int x, int y);
    bool TakePick(PickResult& result);

public:
    bool updateDisplay = false;
    // dark lines at the edges of the opaque objects, see SetOutlines
    bool outlines = true;
    bool displayInitiated = false;
    // true while the simulation FBO holds a valid result for the current view
    bool simCacheValid = false;

protected:
    void InitShaders();
    void CreateDisplayFbos();
    bool CreateGBufferFbo(
        unsigned int& fbo,
        unsigned int& colTex,
        unsigned int& posTex,
        unsigned int& normTex,
        unsigned int& rboDepthStencil
    );
    void CreateSsaoFbos();
    void DeleteGBufferFbo(GBufferFbo& gbuf);
    void BlitGBuffer(unsigned int fromFbo, unsigned int toFbo);
    void CreateFboQuad();
    void SetupVertexAttribs() const;
    void CreateGBufTex(
        GLenum texUnit,
        GLint intFormat,
        GLenum format,
        GLenum type,
        GLuint& texid,
        int width = -1,
        int height = -1
    );
    void UniformHemisphere(vec3& randVec);
    void UniformCircle(vec3& randVec);

private:
    void UpdateCameraView(const SoCamera& camera);
    void UpdateCameraProjection(const SoCamera& camera);

    void UpdateViewMatrix();
    void UpdateProjectionMatrix();

protected:
    // shaders
    Shader shader3D, shaderInv3D, shaderFlat, shaderSimFbo;
    Shader shaderGeom, shaderSSAO, shaderSSAOLighting, shaderSSAOBlur;
    Shader shaderGeomCloser;
    Shader shaderStock;
    Shader shaderOverlay;
    Shader shaderLinePath;
    Shader shaderClearGeom;

    vec3 lightColor = {0.5f, 0.6f, 0.7f};
    vec3 lightPos = {20.0f, 20.0f, 10.0f};
    vec3 ambientCol = {0.2f, 0.2f, 0.25f};
    vec4 pathLineColor = {0.0f, 0.9f, 0.0f, 1.0};
    vec3 pathLineColorPassed = {0.9f, 0.3f, 0.3f};

    mat4x4 mMatLookAt;
    StockObject mlightObject;

    int mWidth = -1;
    int mHeight = -1;

    std::mt19937 generator;
    std::uniform_real_distribution<float> distr01;

    bool mCameraPerspective = true;
    float mCameraHeightAngle = std::numbers::pi / 4;
    float mCameraHeight = 100.0f;
    float mCameraNearDistance = 1.0f;
    float mCameraFarDistance = 100.0f;
    float mMaxStockDimension = 100.0f;

    SbVec3f mCameraPosition;
    SbRotation mCameraOrientation;

    // the transparent layers, see StartTransparentLayer: the G-buffer of a layer, with the depth
    // of the layer and of the one before alternating in two textures, and the lit layers
    unsigned int mTransFbo = 0;
    unsigned int mTransColTexture = 0;
    unsigned int mTransPosTexture = 0;
    unsigned int mTransNormTexture = 0;
    unsigned int mPeelDepth[2] = {0, 0};
    unsigned int mLayerFbo = 0;
    unsigned int mLayerTexture[TransparentLayers] = {};
    int mLayer = -1;      // the layer being drawn, -1 for the opaque objects
    int mLayerCount = 0;  // layers drawn
    bool mTransparentLayer = false;
    bool CreateTransparentFbos();
    void SetPeelState(unsigned int shaderId);
    float mOpacity = 1;
    void ComposeTransparentLayer();
    PickResult mPick;
    void ReadPick(unsigned int fbo, bool transparent);

    unsigned int mFbo = 0;
    unsigned int mFboColTexture = 0;
    unsigned int mFboPosTexture = 0;
    unsigned int mFboNormTexture = 0;
    unsigned int mRboDepthStencil = 0;
    unsigned int mFboQuadVBO = 0;
    // fbo currently receiving geometry passes, either mFbo or mSimFbo
    unsigned int mTargetFbo = 0;

    // simulation frame buffer, keeps the simulated stock without tool/path/base overlays so it
    // can be updated incrementally while the simulation advances
    bool mSimFboSupported = false;
    unsigned int mSimFbo = 0;
    unsigned int mSimColTexture = 0;
    unsigned int mSimPosTexture = 0;
    unsigned int mSimNormTexture = 0;
    unsigned int mSimRboDepthStencil = 0;

    // copies of the simulation frame buffer at different simulation positions, created on demand
    std::vector<GBufferFbo> mSimCheckpoints;

    // ssao frame buffers
    bool mSsaoValid = false;
    int mSsaoWidth = 1;
    int mSsaoHeight = 1;
    std::vector<Point3D> mSsaoKernel;
    unsigned int mSsaoFbo = 0;
    unsigned int mSsaoBlurFbo = 0;
    unsigned int mFboSsaoTexture = 0;
    unsigned int mFboSsaoBlurTexture = 0;
    unsigned int mFboRandTexture = 0;
    // colors of the operation tags of the stock, a row of texels
    unsigned int mOperationColorTexture = 0;
    std::vector<float> mOperationColors;
};

}  // namespace CAMSimulator
