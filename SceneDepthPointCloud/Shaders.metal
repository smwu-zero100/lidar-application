/*
See LICENSE folder for this sample’s licensing information.

Abstract:
The sample app's shaders.
*/

#include <metal_stdlib>
#include <simd/simd.h>
#import "ShaderTypes.h"

using namespace metal;

// Camera's RGB vertex shader outputs
struct RGBVertexOut {
    float4 position [[position]];
    float2 texCoord;
};

// Particle vertex shader outputs and fragment shader inputs
struct ParticleVertexOut {
    float3 pre_position [[]];
    float4 position [[position]];
    float pointSize [[point_size]];
    float4 color;
};

struct Bbox3dVertexOut{
  float4 position [[position]];  //1
  float4 color;
};

constexpr sampler colorSampler(mip_filter::linear, mag_filter::linear, min_filter::linear);
constant auto yCbCrToRGB = float4x4(float4(+1.0000f, +1.0000f, +1.0000f, +0.0000f),
                                    float4(+0.0000f, -0.3441f, +1.7720f, +0.0000f),
                                    float4(+1.4020f, -0.7141f, +0.0000f, +0.0000f),
                                    float4(-0.7010f, +0.5291f, -0.8860f, +1.0000f));
constant float2 viewVertices[] = { float2(-1, 1), float2(-1, -1), float2(1, 1), float2(1, -1) };
constant float2 viewTexCoords[] = { float2(0, 0), float2(0, 1), float2(1, 0), float2(1, 1) };

/// Retrieves the world position of a specified camera point with depth
static simd_float4 worldPoint(simd_float2 cameraPoint, float depth, matrix_float3x3 cameraIntrinsicsInversed, matrix_float4x4 localToWorld) {
    const auto localPoint = cameraIntrinsicsInversed * simd_float3(cameraPoint, 1) * depth;
    const auto worldPoint = localToWorld * simd_float4(localPoint, 1);
    
    return worldPoint / worldPoint.w;
    //return localPoint;
}
 
static simd_float3 localPoint(simd_float2 cameraPoint, float depth, matrix_float3x3 cameraIntrinsicsInversed, matrix_float4x4 localToWorld) {
    auto localPoint = cameraIntrinsicsInversed * simd_float3(cameraPoint, 1) * depth;
    const auto worldPoint = localToWorld * simd_float4(localPoint, 1);
    
    localPoint[0] = cameraPoint.x; //float(int(cameraPoint[0]).interpolated(from: 0...1920, to: 0...1192)); // 2 + 1920 /2;
    localPoint[1] = cameraPoint.y; //float(int(cameraPoint[1]).interpolated(from: 0...1920, to: 0...1192)); // 2 + 1440 /2;
    
    return localPoint;
}

///  Vertex shader that takes in a 2D grid-point and infers its 3D position in world-space, along with RGB and confidence
vertex void unprojectVertex(uint vertexID [[vertex_id]],
                            constant PointCloudUniforms &uniforms [[buffer(kPointCloudUniforms)]],
                            device ParticleUniforms *particleUniforms [[buffer(kParticleUniforms)]],
                            constant float2 *gridPoints [[buffer(kGridPoints)]],
                            texture2d<float, access::sample> capturedImageTextureY [[texture(kTextureY)]],
                            texture2d<float, access::sample> capturedImageTextureCbCr [[texture(kTextureCbCr)]],
                            texture2d<float, access::sample> depthTexture [[texture(kTextureDepth)]],
                            texture2d<unsigned int, access::sample> confidenceTexture [[texture(kTextureConfidence)]],
                            constant BboxInfo *bboxinfo [[buffer(kBboxInfo)]],
                            device ObstacleInfo *obstacleInfo [[buffer(kObstacleInfo)]],
                            device BBoxMinMaxInfo *bboxMinMaxInfo [[buffer(kMinMaxInfo)]]) {
    
    const auto gridPoint = gridPoints[vertexID];
    const auto currentPointIndex = (uniforms.pointCloudCurrentIndex + vertexID) % uniforms.maxPoints;
    const auto texCoord = gridPoint / uniforms.cameraResolution;
    
    // Sample the depth map to get the depth value
    const auto depth = depthTexture.sample(colorSampler, texCoord).r;
    
    // With a 2D point plus depth, we can now get its 3D position
    const auto position = worldPoint(gridPoint, depth, uniforms.cameraIntrinsicsInversed, uniforms.localToWorld);
    const auto loc_position = localPoint(gridPoint, depth, uniforms.cameraIntrinsicsInversed, uniforms.localToWorld);
    
    // Sample Y and CbCr textures to get the YCbCr color at the given texture coordinate
    const auto ycbcr = float4(capturedImageTextureY.sample(colorSampler, texCoord).r, capturedImageTextureCbCr.sample(colorSampler, texCoord.xy).rg, 1);
    const auto sampledColor = (yCbCrToRGB * ycbcr).rgb;
    //const auto sampledColor =float3(0, 1, 0);
    // Sample the confidence map to get the confidence value
    const auto confidence = confidenceTexture.sample(colorSampler, texCoord).r;
    
    const auto a = 1.61; // 가로비
    const auto b = 1.72; // 세로비
    
    const auto x = bboxinfo[0].x * a; // min_x
    const auto y = bboxinfo[0].y * b; // min_y
    const auto w = bboxinfo[0].w * a; // max_x
    const auto h = bboxinfo[0].h * b; // max_y
    
    // Write the data to the buffer
    if((loc_position.x <= w) && (loc_position.y <= h) && (loc_position.x <= 1920) && (loc_position.y <= 1440)) {
        if((x <= loc_position.x) && (y <= loc_position.y) && (x >= 0) && (y >= 0)) {
            particleUniforms[currentPointIndex].color = simd_float3(255, 0, 0);
            //obstacleInfo[0].position = position.xyz;
            
            // update min mix position info
            if(bboxMinMaxInfo[0].min_x > position.x){
                bboxMinMaxInfo[0].min_x = position.x;
            }else if(bboxMinMaxInfo[0].min_x < position.x){
                bboxMinMaxInfo[0].max_x = position.x;
            }
            
            if(bboxMinMaxInfo[0].min_y > position.y){
                bboxMinMaxInfo[0].min_y = position.y;
            }else if(bboxMinMaxInfo[0].min_y < position.y){
                bboxMinMaxInfo[0].max_y = position.y;
            }
            
            if(bboxMinMaxInfo[0].min_z > position.y){
                bboxMinMaxInfo[0].min_z = position.y;
            }else if(bboxMinMaxInfo[0].min_z < position.z){
                bboxMinMaxInfo[0].max_z = position.z;
            }
            
            obstacleInfo[0].position.x =  (obstacleInfo[0].position.x + position.x);
            obstacleInfo[0].position.y = (obstacleInfo[0].position.y + position.y);
            obstacleInfo[0].position.z = (obstacleInfo[0].position.z + position.z);
            obstacleInfo[0].count = obstacleInfo[0].count + 1;
            obstacleInfo[0].depth = depth;
            
        }else{
            particleUniforms[currentPointIndex].color = sampledColor;
        }
    }else{
        particleUniforms[currentPointIndex].color = sampledColor;
    }
    particleUniforms[currentPointIndex].x = x;
    particleUniforms[currentPointIndex].y = y;
    particleUniforms[currentPointIndex].w = w;
    particleUniforms[currentPointIndex].h = h;
    particleUniforms[currentPointIndex].position = position.xyz;
    particleUniforms[currentPointIndex].depth = depth;

    
    if(currentPointIndex == 0) {
        particleUniforms[currentPointIndex].position.x = obstacleInfo[0].position.x/obstacleInfo[0].count;
        particleUniforms[currentPointIndex].position.y = obstacleInfo[0].position.y/obstacleInfo[0].count;
        particleUniforms[currentPointIndex].position.z = obstacleInfo[0].position.z/obstacleInfo[0].count;
        particleUniforms[currentPointIndex].color = simd_float3(0,255,0);
    }
    
    if(currentPointIndex == 1) { // A
        particleUniforms[currentPointIndex].position.x = bboxMinMaxInfo[0].max_x;
        particleUniforms[currentPointIndex].position.y = bboxMinMaxInfo[0].max_y;
        particleUniforms[currentPointIndex].position.z = bboxMinMaxInfo[0].max_z;
        particleUniforms[currentPointIndex].color = simd_float3(125,62,255);
    }
    
    if(currentPointIndex == 2) { // B
        particleUniforms[currentPointIndex].position.x = bboxMinMaxInfo[0].max_x;
        particleUniforms[currentPointIndex].position.y = bboxMinMaxInfo[0].min_y;
        particleUniforms[currentPointIndex].position.z = bboxMinMaxInfo[0].max_z;
        particleUniforms[currentPointIndex].color = simd_float3(0,0,255);
    }
    
    if(currentPointIndex == 3) { // C
        particleUniforms[currentPointIndex].position.x = bboxMinMaxInfo[0].min_x;
        particleUniforms[currentPointIndex].position.y = bboxMinMaxInfo[0].min_y;
        particleUniforms[currentPointIndex].position.z = bboxMinMaxInfo[0].max_z;
        particleUniforms[currentPointIndex].color = simd_float3(0,0,255);
    }
    
    if(currentPointIndex == 4) { // D
        particleUniforms[currentPointIndex].position.x = bboxMinMaxInfo[0].min_x;
        particleUniforms[currentPointIndex].position.y = bboxMinMaxInfo[0].max_y;
        particleUniforms[currentPointIndex].position.z = bboxMinMaxInfo[0].max_z;
        particleUniforms[currentPointIndex].color = simd_float3(0,0,255);
    }
    
    if(currentPointIndex == 5) { // Q
        particleUniforms[currentPointIndex].position.x = bboxMinMaxInfo[0].max_x;
        particleUniforms[currentPointIndex].position.y = bboxMinMaxInfo[0].max_y;
        particleUniforms[currentPointIndex].position.z = bboxMinMaxInfo[0].min_z;
        particleUniforms[currentPointIndex].color = simd_float3(0,0,255);
    }
    
    if(currentPointIndex == 6) { // S
        particleUniforms[currentPointIndex].position.x = bboxMinMaxInfo[0].max_x;
        particleUniforms[currentPointIndex].position.y = bboxMinMaxInfo[0].min_y;
        particleUniforms[currentPointIndex].position.z = bboxMinMaxInfo[0].min_z;
        particleUniforms[currentPointIndex].color = simd_float3(0,0,255);
    }
    
    if(currentPointIndex == 7) { // S
        particleUniforms[currentPointIndex].position.x = bboxMinMaxInfo[0].min_x;
        particleUniforms[currentPointIndex].position.y = bboxMinMaxInfo[0].max_y;
        particleUniforms[currentPointIndex].position.z = bboxMinMaxInfo[0].min_z;
        particleUniforms[currentPointIndex].color = simd_float3(0,0,255);
    }
    
    if(currentPointIndex == 8) { // S
        particleUniforms[currentPointIndex].position.x = bboxMinMaxInfo[0].min_x;
        particleUniforms[currentPointIndex].position.y = bboxMinMaxInfo[0].min_y;
        particleUniforms[currentPointIndex].position.z = bboxMinMaxInfo[0].min_z;
        particleUniforms[currentPointIndex].color = simd_float3(0,0,255);
    }
    
    
   // particleUniforms[currentPointIndex].color = sampledColor;
    particleUniforms[currentPointIndex].confidence = confidence;
}

vertex Bbox3dVertexOut obstacleDetectionVertex(uint vertexID [[vertex_id]],
                              device BBoxMinMaxInfo &minMaxInfo [[buffer(kMinMaxInfo)]],
                              device Bbox3dInfo *bbox3dInfo [[buffer(kBbox3dInfo)]]) {
    
    Bbox3dVertexOut outVertex;
    /*
     * A - B - C - D - A - Q - R - D - C - T - R - Q - S - B - S - T
     * 0 - 1 - 2 - 3 - 4 - 5 - 6 - 7 - 8 - 9 - 10 - 11 - 12 - 13 - 14 - 15
     */
    
    if(vertexID == 0) {
        outVertex.position.x = -0.025711672; //minMaxInfo.max_x;
        outVertex.position.y = -0.14813556; //minMaxInfo.max_y;
        outVertex.position.z = -0.116113186; //minMaxInfo.max_z;
    }else if (vertexID == 1) {
        outVertex.position.x = -0.025711672; //minMaxInfo.max_x;
        outVertex.position.y = -0.17169493; //minMaxInfo.min_y;
        outVertex.position.z = -0.116113186; //minMaxInfo.max_z;
    }
//    }else if (vertexID == 2) {
//        outVertex.position.x = 0.0; //minMaxInfo.min_x;
//        outVertex.position.y = 0.0; //minMaxInfo.min_y;
//        outVertex.position.z = -1.0; //minMaxInfo.max_z;
//    }else if (vertexID == 3) {
//        outVertex.position.x = 0.0; //minMaxInfo.min_x;
//        outVertex.position.y = -1.0; //minMaxInfo.max_y;
//        outVertex.position.z = -1.0; //minMaxInfo.max_z;
//    }
    
//    switch (int(vertexID)) {
//    case 0: // A
//    case 4:
//        outVertex.position.x = minMaxInfo.max_x;
//        outVertex.position.y = minMaxInfo.max_y;
//        outVertex.position.z = minMaxInfo.max_z;
//        break;
//    case 1: // B
//    case 13:
//        outVertex.position.x = minMaxInfo.max_x;
//        outVertex.position.y = minMaxInfo.min_y;
//        outVertex.position.z = minMaxInfo.max_z;
//        break;
//    case 2: // C
//    case 8:
//        outVertex.position.x = minMaxInfo.min_x;
//        outVertex.position.y = minMaxInfo.min_y;
//        outVertex.position.z = minMaxInfo.max_z;
//        break;
//    case 3: // D
//    case 7:
//        outVertex.position.x = minMaxInfo.min_x;
//        outVertex.position.y = minMaxInfo.max_y;
//        outVertex.position.z = minMaxInfo.max_z;
//        break;
//    case 5: // Q
//    case 11:
//        outVertex.position.x = minMaxInfo.max_x;
//        outVertex.position.y = minMaxInfo.max_y;
//        outVertex.position.z = minMaxInfo.min_z;
//        break;
//    case 14: // S
//    case 12:
//        outVertex.position.x = minMaxInfo.max_x;
//        outVertex.position.y = minMaxInfo.min_y;
//        outVertex.position.z = minMaxInfo.min_z;
//        break;
//    case 9: // T
//    case 15:
//        outVertex.position.x = minMaxInfo.min_x;
//        outVertex.position.y = minMaxInfo.max_y;
//        outVertex.position.z = minMaxInfo.min_z;
//        break;
//    case 6: // R
//    case 10:
//        outVertex.position.x = minMaxInfo.min_x;
//        outVertex.position.y = minMaxInfo.min_y;
//        outVertex.position.z = minMaxInfo.min_z;
//        break;
//    }
    
    bbox3dInfo[vertexID].vertexId = vertexID;
    bbox3dInfo[vertexID].position.x = outVertex.position.x;
    bbox3dInfo[vertexID].position.y = outVertex.position.y;
    bbox3dInfo[vertexID].position.z = outVertex.position.z;
    bbox3dInfo[vertexID].color = float3(0,0,255);
    outVertex.color = float4(0,0,255,0);
    
    return outVertex;
}

fragment half4 obstacleDetectionFragment(Bbox3dVertexOut inFrag [[stage_in]])
{
//  return half4(1, 0, 0, 1);
  
    return half4(inFrag.color);
};

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
    
    // convert and save the color back to the buffer
    const float3 sampledColor = (yCbCrToRGB * ycbcr).rgb;
    return float4(sampledColor, 1) * visibility;
}
//0220
vertex float4 bboxVertex(const device packed_float2* vertices [[ buffer(0) ]],
                       unsigned int vertexId [[ vertex_id ]])
{

    return float4(vertices[vertexId], 0.0, 1.0);
    
}

fragment half4 bboxFragment() {
  return half4(0, 1, 0, 1);
}
vertex ParticleVertexOut particleVertex(uint vertexID [[vertex_id]],
                                        constant PointCloudUniforms &uniforms [[buffer(kPointCloudUniforms)]],
                                        device ParticleUniforms *particleUniforms [[buffer(kParticleUniforms)]]) {
    
    const auto currentPointIndex = (uniforms.pointCloudCurrentIndex + vertexID) % uniforms.maxPoints;
    // get point data
    const auto particleData = particleUniforms[vertexID];
    const auto position = particleData.position;
    const auto confidence = particleData.confidence;
    const auto sampledColor = particleData.color;
    const auto visibility = confidence >= uniforms.confidenceThreshold;

    // animate and project the point
    float4 projectedPosition = uniforms.viewProjectionMatrix * float4(position, 1.0);
    const float pointSize = max(uniforms.particleSize / max(1.0, projectedPosition.z), 2.0);
    projectedPosition /= projectedPosition.w;
    
    //write
    particleUniforms[currentPointIndex].particlebuffer_position = projectedPosition;
    //particleUniforms[currentPointIndex].position = projectedPosition.xyz;
    //particleUniforms[currentPointIndex].color = sampledColor;
    //particleUniforms[currentPointIndex].confidence = confidence;
    
    // prepare for output
    ParticleVertexOut out;
    out.pre_position = position;
    out.position = projectedPosition;
    out.pointSize = pointSize;
    out.color = float4(sampledColor, visibility);
    
    return out;
}

fragment float4 particleFragment(ParticleVertexOut in [[stage_in]],
                                 const float2 coords [[point_coord]]) {
    // we draw within a circle
    const float distSquared = length_squared(coords - float2(0.5));
    if (in.color.a == 0 || distSquared > 0.25) {
        discard_fragment();
    }
    
    return in.color;
}
