/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Types and enums that are shared between shaders and the host app code.
*/

#ifndef ShaderTypes_h
#define ShaderTypes_h

#include <simd/simd.h>

enum TextureIndices {
    kTextureY = 0,
    kTextureCbCr = 1,
    kTextureDepth = 2,
    kTextureConfidence = 3
};

enum BufferIndices {
    kPointCloudUniforms = 0,
    kParticleUniforms = 1,
    kGridPoints = 2,
    //0330
    kBboxUniforms = 3,
    kBboxInfo = 4,
    kObstacleInfo = 5,
    kBbox3dInfo = 6,
    kMinMaxInfo = 7
};

struct RGBUniforms {
    matrix_float3x3 viewToCamera;
    float viewRatio;
    float radius;
};

struct PointCloudUniforms {
    matrix_float4x4 viewProjectionMatrix;
    matrix_float4x4 localToWorld;
    matrix_float3x3 cameraIntrinsicsInversed;
    simd_float2 cameraResolution;
    
    float particleSize;
    int maxPoints;
    int pointCloudCurrentIndex;
    int confidenceThreshold;
};

struct ParticleUniforms {
    simd_float2 worldpoint_localposition;
    simd_float4 particlebuffer_position;
    simd_float3 particlebuffer_localposition;
    simd_float3 position;
    simd_float3 color;
    float x;
    float y;
    float w;
    float h;
    float confidence;
    float depth;
};

struct ObstacleInfo {
    simd_float3 position;
    simd_float3 color;
    int count;
    float depth;
};

struct BboxInfo {
    float x;
    float y;
    float w;
    float h;
};

struct BBoxMinMaxInfo {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
};

struct Bbox3dInfo {
    simd_float3 position;
    simd_float3 color;
    int count;
    float depth;
};


#endif /* ShaderTypes_h */
