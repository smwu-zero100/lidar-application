#include <metal_stdlib>
#include <simd/simd.h>
#import "ShaderTypes.h"

using namespace metal;

struct RGBVertexOut {
    float4 position [[position]];
    float2 texCoord;
};

struct ParticleVertexOut {
    float4 position [[position]];
    float pointSize [[point_size]];
    float4 color;
};

constexpr sampler colorSampler(mip_filter::linear, mag_filter::linear, min_filter::linear);
constant auto yCbCrToRGB = float4x4(float4(+1.0000f, +1.0000f, +1.0000f, +0.0000f),
                                    float4(+0.0000f, -0.3441f, +1.7720f, +0.0000f),
                                    float4(+1.4020f, -0.7141f, +0.0000f, +0.0000f),
                                    float4(-0.7010f, +0.5291f, -0.8860f, +1.0000f));
constant float2 viewVertices[] = { float2(-1, 1), float2(-1, -1), float2(1, 1), float2(1, -1) };
constant float2 viewTexCoords[] = { float2(0, 0), float2(0, 1), float2(1, 0), float2(1, 1) };

static simd_float4 worldPoint(simd_float2 cameraPoint, float depth, matrix_float3x3 cameraIntrinsicsInversed, matrix_float4x4 localToWorld) {
    const auto localPoint = cameraIntrinsicsInversed * simd_float3(cameraPoint, 1) * depth;
    const auto worldPoint = localToWorld * simd_float4(localPoint, 1);
    
    return worldPoint / worldPoint.w;
}
 
static simd_float3 localPoint(simd_float2 cameraPoint, float depth, matrix_float3x3 cameraIntrinsicsInversed, matrix_float4x4 localToWorld) {
    auto localPoint = cameraIntrinsicsInversed * simd_float3(cameraPoint, 1) * depth;
    localPoint[0] = cameraPoint.x;
    localPoint[1] = cameraPoint.y;
    
    return localPoint;
}

vertex void unprojectVertex(uint vertexID [[vertex_id]],
                            constant PointCloudUniforms &uniforms [[buffer(kPointCloudUniforms)]],
                            device ParticleUniforms *particleUniforms [[buffer(kParticleUniforms)]],
                            constant float2 *gridPoints [[buffer(kGridPoints)]],
                            texture2d<float, access::sample> capturedImageTextureY [[texture(kTextureY)]],
                            texture2d<float, access::sample> capturedImageTextureCbCr [[texture(kTextureCbCr)]],
                            texture2d<float, access::sample> depthTexture [[texture(kTextureDepth)]],
                            texture2d<unsigned int, access::sample> confidenceTexture [[texture(kTextureConfidence)]],
                            constant BboxInfo *bboxinfo [[buffer(kBboxInfo)]]) {
    
    const auto gridPoint = gridPoints[vertexID];
    const auto currentPointIndex = (uniforms.pointCloudCurrentIndex + vertexID) % uniforms.maxPoints;
    const auto texCoord = gridPoint / uniforms.cameraResolution;
    const auto depth = depthTexture.sample(colorSampler, texCoord).r;
    const auto worldPosition = worldPoint(gridPoint, depth, uniforms.cameraIntrinsicsInversed, uniforms.localToWorld);
    const auto loc_position = localPoint(gridPoint, depth, uniforms.cameraIntrinsicsInversed, uniforms.localToWorld);
    const auto ycbcr = float4(capturedImageTextureY.sample(colorSampler, texCoord).r, capturedImageTextureCbCr.sample(colorSampler, texCoord.xy).rg, 1);
    const auto sampledColor = (yCbCrToRGB * ycbcr).rgb;
    const auto confidence = confidenceTexture.sample(colorSampler, texCoord).r;
    
    const auto horizontalRatioForViewToCamera = 1.61; // camera: 1920, renderer: 1440
    const auto verticalRatioForViewToCamera = 1.72; // camera: 1194, renderer: 834
    
    const auto bboxMinX_Renderer = bboxinfo[0].x * horizontalRatioForViewToCamera; // min_x
    const auto bboxMinY_Renderer = bboxinfo[0].y * verticalRatioForViewToCamera; // min_y
    const auto bboxMaxX_Renderer = bboxinfo[0].w * horizontalRatioForViewToCamera; // max_x
    const auto bboxMaxY_Renderer = bboxinfo[0].h * verticalRatioForViewToCamera; // max_y
    
    // Write the data to the buffer
    if((loc_position.x <= bboxMaxX_Renderer) && (loc_position.y <= (1440-bboxMinY_Renderer)) && (loc_position.x <= 1920) && (loc_position.y <= 1440)) {
        if((bboxMinX_Renderer <= loc_position.x) && ((1440-bboxMaxY_Renderer) <= loc_position.y) && (bboxMinX_Renderer >= 0) && (bboxMinY_Renderer >= 0)) {
            particleUniforms[currentPointIndex].color = simd_float3(255, 0, 0);
        }else{
            particleUniforms[currentPointIndex].color = sampledColor;
        }
    }else{
        particleUniforms[currentPointIndex].color = sampledColor;
    }
    particleUniforms[currentPointIndex].x = bboxMinX_Renderer;
    particleUniforms[currentPointIndex].y = bboxMinY_Renderer;
    particleUniforms[currentPointIndex].w = bboxMaxX_Renderer;
    particleUniforms[currentPointIndex].h = bboxMaxY_Renderer;
    particleUniforms[currentPointIndex].position = worldPosition.xyz;
    particleUniforms[currentPointIndex].depth = depth;
    particleUniforms[currentPointIndex].confidence = confidence;
}

vertex RGBVertexOut rgbVertex(uint vertexID [[vertex_id]],
                              constant RGBUniforms &uniforms [[buffer(0)]]) {
    const float3 texCoord = float3(viewTexCoords[vertexID], 1) * uniforms.viewToCamera;
    
    RGBVertexOut out;
    out.position = float4(viewVertices[vertexID], 0, 1);
    out.texCoord = texCoord.xy;
    
    return out;
}

fragment float4 rgbFragment(RGBVertexOut in [[stage_in]],
                            constant RGBUniforms &uniforms [[buffer(0)]],
                            texture2d<float, access::sample> capturedImageTextureY [[texture(kTextureY)]],
                            texture2d<float, access::sample> capturedImageTextureCbCr [[texture(kTextureCbCr)]]) {
    
    const float2 offset = (in.texCoord - 0.5) * float2(1, 1 / uniforms.viewRatio) * 2;
    const float visibility = saturate(uniforms.radius * uniforms.radius - length_squared(offset));
    const float4 ycbcr = float4(capturedImageTextureY.sample(colorSampler, in.texCoord.xy).r, capturedImageTextureCbCr.sample(colorSampler, in.texCoord.xy).rg, 1);
    
    const float3 sampledColor = (yCbCrToRGB * ycbcr).rgb;
    return float4(sampledColor, 1) * visibility;
}
vertex ParticleVertexOut particleVertex(uint vertexID [[vertex_id]],
                                        constant PointCloudUniforms &uniforms [[buffer(kPointCloudUniforms)]],
                                        device ParticleUniforms *particleUniforms [[buffer(kParticleUniforms)]]) {
    
    const auto particleData = particleUniforms[vertexID];
    const auto position = particleData.position;
    const auto confidence = particleData.confidence;
    const auto sampledColor = particleData.color;
    const auto visibility = confidence >= uniforms.confidenceThreshold;
    float4 projectedPosition = uniforms.viewProjectionMatrix * float4(position, 1.0);
    const float pointSize = max(uniforms.particleSize / max(1.0, projectedPosition.z), 2.0);
       
    ParticleVertexOut out;
    out.position = projectedPosition;
    out.pointSize = pointSize;
    out.color = float4(sampledColor, visibility);
    
    return out;
}

fragment float4 particleFragment(ParticleVertexOut in [[stage_in]],
                                 const float2 coords [[point_coord]]) {

    const float distSquared = length_squared(coords - float2(0.5));
    if (in.color.a == 0 || distSquared > 0.25) {
        discard_fragment();
    }
    
    return in.color;
}
