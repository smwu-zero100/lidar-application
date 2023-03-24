/*
See LICENSE folder for this sample’s licensing information.

Abstract:
The host app renderer.
*/

import Metal
import MetalKit
import SceneKit
import ARKit
import GLKit
import simd
import Foundation

final class Renderer {
    // Maximum number of points we store in the point cloud
    private let maxPoints = 1000
    //100_000_00
    // Number of sample points on the grid
    private let numGridPoints = 1000
    // Particle's size in pixels
    private let particleSize: Float = 10
    // We only use landscape orientation in this app
    private let orientation = UIInterfaceOrientation.landscapeRight
    // Camera's threshold values for detecting when the camera moves so that we can accumulate the points
    private let cameraRotationThreshold = cos(2 * .degreesToRadian)
    //cameraRotationThreshold : 0.99939084
    private let cameraTranslationThreshold: Float = pow(0.02, 2)   // (meter-squared)
    //cameraTranslationThreshold : 0.0004
    // The max number of command buffers in flight
    private let maxInFlightBuffers = 7
    //3
    
    //0306
    var bound : CGRect
    
    private lazy var rotateToARCamera = Self.makeRotateToARCameraMatrix(orientation: orientation)
    private let session: ARSession
    
    // Metal objects and textures
    private let device: MTLDevice
    private let library: MTLLibrary
    //private let renderDestination: RenderDestinationProvider
    private let sceneView: ARSCNView
    private let relaxedStencilState: MTLDepthStencilState
    private let depthStencilState: MTLDepthStencilState
    private let commandQueue: MTLCommandQueue
    private lazy var unprojectPipelineState = makeUnprojectionPipelineState()!
    private lazy var rgbPipelineState = makeRGBPipelineState()!
    private lazy var particlePipelineState = makeParticlePipelineState()!
    private lazy var objectDetectionPipelineState = makeObjectDetectionPipelineState()!;
    //0220
    private lazy var bboxPipelineState = makeBboxPipelineState()!
    // texture cache for captured image
    private lazy var textureCache = makeTextureCache()
    private var capturedImageTextureY: CVMetalTexture?
    private var capturedImageTextureCbCr: CVMetalTexture?
    private var depthTexture: CVMetalTexture?
    private var confidenceTexture: CVMetalTexture?
    
    // Multi-buffer rendering pipeline
    private let inFlightSemaphore: DispatchSemaphore
    private var currentBufferIndex = 0
    
    // The current viewport size
    private var viewportSize = CGSize()
    // The grid of sample points
    private lazy var gridPointsBuffer = MetalBuffer<Float2>(device: device,
                                                            array: makeGridPoints(),
                                                            index: kGridPoints.rawValue, options: [])
    // RGB buffer
    private lazy var rgbUniforms: RGBUniforms = {
        var uniforms = RGBUniforms()
        uniforms.radius = rgbRadius
        uniforms.viewToCamera.copy(from: viewToCamera)
        uniforms.viewRatio = Float(viewportSize.width / viewportSize.height)
        return uniforms
    }()
    private var rgbUniformsBuffers = [MetalBuffer<RGBUniforms>]()
    // Point Cloud buffer
    private lazy var pointCloudUniforms: PointCloudUniforms = {
        var uniforms = PointCloudUniforms()
        uniforms.maxPoints = Int32(maxPoints)
        uniforms.confidenceThreshold = Int32(confidenceThreshold)
        uniforms.particleSize = particleSize
        uniforms.cameraResolution = cameraResolution
        return uniforms
    }()
    private var pointCloudUniformsBuffers = [MetalBuffer<PointCloudUniforms>]()
    // Particles buffer
    var particlesBuffer: MetalBuffer<ParticleUniforms>
    var objectDetectionBuffer: MetalBuffer<BboxInfo>
    var centeroidBuffer: MetalBuffer<ObstacleInfo>
    var bboxMinMaxBuffer: MetalBuffer<BBoxMinMaxInfo>
    var objectDetection3dBuffer: MetalBuffer<Bbox3dInfo>
    //0303
    
    private var currentPointIndex = 0
    var currentPointCount = 0
    var olderNode = SCNNode(geometry: SCNBox(width: 0, height: 0, length: 0, chamferRadius: 0))
    
    // Camera data
    private var sampleFrame: ARFrame { session.currentFrame! }
    private lazy var cameraResolution = Float2(Float(sampleFrame.camera.imageResolution.width), Float(sampleFrame.camera.imageResolution.height))
    // cameraResolution : SIMD2<Float>(1920.0, 1440.0)
    private lazy var viewToCamera = sampleFrame.displayTransform(for: orientation, viewportSize: viewportSize).inverted()
    
    private lazy var lastCameraTransform = sampleFrame.camera.transform
    
    var count = 0;
    var localDepth : Float = 100.0;
    var localDepthMid : Float = 0.0;
    var localMin : SIMD3<Float> = simd_float3(0, 0, 0);
    var localMax : SIMD3<Float> = simd_float3(0, 0, 0);
    var centerPoint : SIMD3<Float> = simd_float3(0, 0, 0);
    var realCenterPoint : SIMD3<Float> = simd_float3(0, 0, 0);
    var maxOutliarDistance: Float = 0.03;
    
    var filteredPoints: [SIMD3<Float>] = [];
    var filteredPoints_2: [SIMD3<Float>] = [];
    
    var _min : Float = 100.0
    var _max : Float = -100.0
    
    var locminx : Float = 1000.0
    var locmaxx : Float = -100.0
    var locminy : Float = 1000.0
    var locmaxy : Float = -100.0
    
    var posminx : Float = 1000.0
    var posmaxx : Float = -100.0
    var posminy : Float = 1000.0
    var posmaxy : Float = -100.0
    
    // interfaces
    var confidenceThreshold = 1 {
        didSet {
            // apply the change for the shader
            pointCloudUniforms.confidenceThreshold = Int32(confidenceThreshold)
        }
    }
    
    var rgbRadius: Float = 0 {
        didSet {
            // apply the change for the shader
            rgbUniforms.radius = rgbRadius
        }
    }
    
    // renderDestination -> mtkView
    init(session: ARSession, metalDevice device: MTLDevice, sceneView: ARSCNView) {
        self.session = session
        self.device = device
        //self.renderDestination = renderDestination
        self.sceneView = sceneView
        self.bound = CGRect(x: 0, y: 0, width: 0, height: 0)
        
        library = device.makeDefaultLibrary()!
        //commandQueue = device.makeCommandQueue()!
        commandQueue = sceneView.commandQueue!
        
        // initialize our buffers
        for _ in 0 ..< maxInFlightBuffers {
            rgbUniformsBuffers.append(.init(device: device, count: 1, index: 0))
            pointCloudUniformsBuffers.append(.init(device: device, count: 1, index: kPointCloudUniforms.rawValue))
        }
        particlesBuffer = .init(device: device, count: maxPoints, index: kParticleUniforms.rawValue)
        //currentBufferIndex: 0과 2만 반복
        objectDetectionBuffer = .init(device: device, count: 10, index: kBboxInfo.rawValue )
        objectDetection3dBuffer = .init(device: device, count: 2, index: kBbox3dInfo.rawValue)
        bboxMinMaxBuffer = .init(device: device, count: 1, index: kMinMaxInfo.rawValue)
        centeroidBuffer = .init(device: device, count: 1, index: kObstacleInfo.rawValue )
        
        // init MinMaxbuffer
        bboxMinMaxBuffer[0].min_x = 100.0;
        bboxMinMaxBuffer[0].max_x = -100.0;
        bboxMinMaxBuffer[0].min_y = 100.0;
        bboxMinMaxBuffer[0].max_y = -100.0;
        bboxMinMaxBuffer[0].min_z = 100.0;
        bboxMinMaxBuffer[0].max_z = -100.0;
        
        // init centroidbuffer
        centeroidBuffer[0].position.x = 0.0;
        centeroidBuffer[0].position.y = 0.0;
        centeroidBuffer[0].position.z = 0.0;
        centeroidBuffer[0].count = 0;
        centeroidBuffer[0].depth = 100.0;
        
        objectDetection3dBuffer[0].position.x = 0;
        objectDetection3dBuffer[0].position.y = 0;
        objectDetection3dBuffer[0].position.z = 0;
        
        
        objectDetection3dBuffer[1].position.x = 1;
        objectDetection3dBuffer[1].position.y = 0;
        objectDetection3dBuffer[1].position.z = 0;
        
       
        olderNode.name = "3d bbox"
        olderNode.position = SCNVector3(0, 0, 0)
        sceneView.scene.rootNode.addChildNode(olderNode)
        
        
        
        // rbg does not need to read/write depth
        let relaxedStateDescriptor = MTLDepthStencilDescriptor()
        relaxedStencilState = device.makeDepthStencilState(descriptor: relaxedStateDescriptor)!
        
        // setup depth test for point cloud
        let depthStateDescriptor = MTLDepthStencilDescriptor()
        depthStateDescriptor.depthCompareFunction = .greaterEqual
        depthStateDescriptor.isDepthWriteEnabled = true
        depthStencilState = device.makeDepthStencilState(descriptor: depthStateDescriptor)!

        inFlightSemaphore = DispatchSemaphore(value: maxInFlightBuffers)
    }
    
    func drawRectResized(size: CGSize) {
        viewportSize = size
    }
    
    func addPoint(points: [vector_float3]) {
        
    }
   
    private func updateCapturedImageTextures(frame: ARFrame) {
        // Create two textures (Y and CbCr) from the provided frame's captured image
        let pixelBuffer = frame.capturedImage
        //print("pixelBuffer : \(pixelBuffer)")
        guard CVPixelBufferGetPlaneCount(pixelBuffer) >= 2 else {
            return
        }
        
        capturedImageTextureY = makeTexture(fromPixelBuffer: pixelBuffer, pixelFormat: .r8Unorm, planeIndex: 0)
        //print("capturedImageTextureY : \(capturedImageTextureY)")
        capturedImageTextureCbCr = makeTexture(fromPixelBuffer: pixelBuffer, pixelFormat: .rg8Unorm, planeIndex: 1)
    }
    
    private func updateDepthTextures(frame: ARFrame) -> Bool {
        guard let depthMap = frame.sceneDepth?.depthMap,
            let confidenceMap = frame.sceneDepth?.confidenceMap else {
                return false
        }
        
        depthTexture = makeTexture(fromPixelBuffer: depthMap, pixelFormat: .r32Float, planeIndex: 0)
        confidenceTexture = makeTexture(fromPixelBuffer: confidenceMap, pixelFormat: .r8Uint, planeIndex: 0)
        
        return true
    }
    
    private func update(frame: ARFrame) {
        // frame dependent info
        let camera = frame.camera
        let cameraIntrinsicsInversed = camera.intrinsics.inverse
        let viewMatrix = camera.viewMatrix(for: orientation)
        let viewMatrixInversed = viewMatrix.inverse
        let projectionMatrix = camera.projectionMatrix(for: orientation, viewportSize: viewportSize, zNear: 0.001, zFar: 0)
        pointCloudUniforms.viewProjectionMatrix = projectionMatrix * viewMatrix
        pointCloudUniforms.localToWorld = viewMatrixInversed * rotateToARCamera
        pointCloudUniforms.cameraIntrinsicsInversed = cameraIntrinsicsInversed
    }

    func draw() {
//        sceneView.scene.rootNode.enumerateChildNodes { (node, stop) in
//            node.removeFromParentNode()
//        }
        
        // init MinMaxbuffer
//        bboxMinMaxBuffer[0].min_x = 10.0;
//        bboxMinMaxBuffer[0].max_x = -10.0;
//        bboxMinMaxBuffer[0].min_y = 10.0;
//        bboxMinMaxBuffer[0].max_y = -10.0;
//        bboxMinMaxBuffer[0].min_z = 10.0;
//        bboxMinMaxBuffer[0].max_z = -10.0;
        
//        // init centroidbuffer
//        centeroidBuffer[0].position.x = 0.0;
//        centeroidBuffer[0].position.y = 0.0;
//        centeroidBuffer[0].position.z = 0.0;
//        centeroidBuffer[0].count = 0;
//        centeroidBuffer[0].depth = 100.0;
        
        
//        // init centroidbuffer
        guard let currentFrame = session.currentFrame,
            let commandBuffer = commandQueue.makeCommandBuffer(),
            let renderEncoder = sceneView.currentRenderCommandEncoder else {
                  return
        }
        
        _ = inFlightSemaphore.wait(timeout: DispatchTime.distantFuture)
        commandBuffer.addCompletedHandler { [weak self] commandBuffer in
            if let self = self {
                self.inFlightSemaphore.signal()
            }
        }

        // update frame data
        update(frame: currentFrame)
        updateCapturedImageTextures(frame: currentFrame)
        
        
        // handle buffer rotating
        currentBufferIndex = (currentBufferIndex + 1) % maxInFlightBuffers
        pointCloudUniformsBuffers[currentBufferIndex][0] = pointCloudUniforms
        
       // print("shouldAccumulate(frame: currentFrame): \(shouldAccumulate(frame: currentFrame))")

        if shouldAccumulate(frame: currentFrame), updateDepthTextures(frame: currentFrame) {
            accumulatePoints(frame: currentFrame, commandBuffer: commandBuffer, renderEncoder: renderEncoder)
        }
    
//
//        let width = sqrt((bboxMinMaxBuffer[0].max_x - bboxMinMaxBuffer[0].min_x) * (bboxMinMaxBuffer[0].max_x - bboxMinMaxBuffer[0].min_x));
//        let height = sqrt((bboxMinMaxBuffer[0].max_y - bboxMinMaxBuffer[0].min_y) * (bboxMinMaxBuffer[0].max_y - bboxMinMaxBuffer[0].min_y));
//        let length = sqrt((bboxMinMaxBuffer[0].max_z - bboxMinMaxBuffer[0].min_z) * (bboxMinMaxBuffer[0].max_z - bboxMinMaxBuffer[0].min_z));

//        print("---------------------------")
////        print(width, height, length)
//        print(centeroidBuffer[0].position)
//        print(centeroidBuffer[0].count)
//        
//        print(centeroidBuffer[0].position.x / Float(centeroidBuffer[0].count))
//        print(centeroidBuffer[0].position.y / Float(centeroidBuffer[0].count))
//        print(centeroidBuffer[0].position.z / Float(centeroidBuffer[0].count))
////        print(particlesBuffer[0].position)
        
        particlesBuffer[0].position.x = centeroidBuffer[0].position.x / Float(centeroidBuffer[0].count)
        particlesBuffer[0].position.y = centeroidBuffer[0].position.y / Float(centeroidBuffer[0].count)
        particlesBuffer[0].position.z = centeroidBuffer[0].position.z / Float(centeroidBuffer[0].count)
    
        
        // check and render rgb camera image
        if rgbUniforms.radius > 0 {
            var retainingTextures = [capturedImageTextureY, capturedImageTextureCbCr]
            commandBuffer.addCompletedHandler { buffer in
                retainingTextures.removeAll()
            }
            rgbUniformsBuffers[currentBufferIndex][0] = rgbUniforms
   
            
            renderEncoder.setDepthStencilState(relaxedStencilState)
            renderEncoder.setRenderPipelineState(rgbPipelineState)

            renderEncoder.setVertexBuffer(rgbUniformsBuffers[currentBufferIndex])
            renderEncoder.setFragmentBuffer(rgbUniformsBuffers[currentBufferIndex])
            renderEncoder.setFragmentTexture(CVMetalTextureGetTexture(capturedImageTextureY!), index: Int(kTextureY.rawValue))
            renderEncoder.setFragmentTexture(CVMetalTextureGetTexture(capturedImageTextureCbCr!), index: Int(kTextureCbCr.rawValue))
            renderEncoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)
        }
        
        // render particles
        renderEncoder.setDepthStencilState(depthStencilState)
        renderEncoder.setRenderPipelineState(particlePipelineState)
        renderEncoder.setVertexBuffer(pointCloudUniformsBuffers[currentBufferIndex])
        //currentBufferIndex : 0,1,2,3 반복
        renderEncoder.setVertexBuffer(particlesBuffer)

        renderEncoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: currentPointCount)
        //renderEncoder.endEncoding()
            
        //commandBuffer.present(renderDestination.currentDrawable!)
        commandBuffer.commit()
        
        count = 0;
        localDepth = 100.0;
        localDepthMid = 100.0;
        localMax = SIMD3(-100, -100, -100);
        localMin = SIMD3(100, 100, 100);
        centerPoint = SIMD3(0, 0, 0);
        realCenterPoint = SIMD3(0, 0, 0);
        filteredPoints = [];
        
        _min = 100.0
        _max = -100.0
//
//        var locminx : Float = 1000.0
//        var locmaxx : Float = -100.0
//        var locminy : Float = 1000.0
//        var locmaxy : Float = -100.0
//
//        var posminx : Float = 1000.0
//        var posmaxx : Float = -100.0
//        var posminy : Float = 1000.0
//        var posmaxy : Float = -100.0
        
        
        for i in 0..<currentPointCount {
            let point = particlesBuffer[i]
            
            if(point.color == simd_float3(255, 0, 0)){
                
                if(point.color == simd_float3(255, 0, 0)){
                    count = count + 1
                    centerPoint = centerPoint + simd_float3(point.position.x,point.position.y, point.position.z)
                    
                    if(point.depth < _min){
                        _min = point.depth
                    }
                    if(point.depth > _max){
                        _max = point.depth
                    }
                    
                    if(point.x < locminx){
                        locminx = point.x
                    }
                    if(point.x > locmaxx){
                        locmaxx = point.x
                        
                    }
                    if(point.y < locminy){
                        locminy = point.y
                    }
                    if(point.y > locmaxy){
                        locmaxy = point.y
                    }
                    
                    if(point.position.x < posminx){
                        posminx = point.position.x
                    }
                    if(point.position.x > posmaxx){
                        posmaxx = point.position.x
                        
                    }
                    if(point.position.y < posminy){
                        posminy = point.position.y
                    }
                    if(point.position.y > posmaxy){
                        posmaxy = point.position.y
                    }
                }
                
            }
        }
        
        centerPoint = centerPoint/Float(count);
        
        localDepthMid = localDepthMid / Float(count);
        
    }
    
    private func shouldAccumulate(frame: ARFrame) -> Bool {
        let cameraTransform = frame.camera.transform
        return currentPointCount == 0
            || dot(cameraTransform.columns.2, lastCameraTransform.columns.2) <= cameraRotationThreshold
            || distance_squared(cameraTransform.columns.3, lastCameraTransform.columns.3) >= cameraTranslationThreshold
    }
    
    private func accumulatePoints(frame: ARFrame, commandBuffer: MTLCommandBuffer, renderEncoder: MTLRenderCommandEncoder) {
        pointCloudUniforms.pointCloudCurrentIndex = Int32(currentPointIndex)
        
        var retainingTextures = [capturedImageTextureY, capturedImageTextureCbCr, depthTexture, confidenceTexture]
        commandBuffer.addCompletedHandler { buffer in
            retainingTextures.removeAll()
        }

        renderEncoder.setDepthStencilState(relaxedStencilState)
        renderEncoder.setRenderPipelineState(unprojectPipelineState)
        renderEncoder.setVertexBuffer(pointCloudUniformsBuffers[currentBufferIndex])
        //3d bbox
        //renderEncoder.setVertexBuffer(objectDetection3dBuffer)
        //2d bbox
        renderEncoder.setVertexBuffer(objectDetectionBuffer)
        //2d bbox min max info
        renderEncoder.setVertexBuffer(bboxMinMaxBuffer)
        //cenroid point info
        renderEncoder.setVertexBuffer(centeroidBuffer)
        
        renderEncoder.setVertexBuffer(particlesBuffer)
        renderEncoder.setVertexBuffer(gridPointsBuffer)
        renderEncoder.setVertexTexture(CVMetalTextureGetTexture(capturedImageTextureY!), index: Int(kTextureY.rawValue))
        renderEncoder.setVertexTexture(CVMetalTextureGetTexture(capturedImageTextureCbCr!), index: Int(kTextureCbCr.rawValue))
        renderEncoder.setVertexTexture(CVMetalTextureGetTexture(depthTexture!), index: Int(kTextureDepth.rawValue))
        renderEncoder.setVertexTexture(CVMetalTextureGetTexture(confidenceTexture!), index: Int(kTextureConfidence.rawValue))
        renderEncoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: gridPointsBuffer.count)

        currentPointIndex = (currentPointIndex + gridPointsBuffer.count) % maxPoints
        currentPointCount = min(currentPointCount + gridPointsBuffer.count, maxPoints)
        lastCameraTransform = frame.camera.transform
    
    }
    
    public func exportMesh(){
        
    }
}

// MARK: - Metal Helpers

private extension Renderer {
    
    // 3d bbox
    func makeObjectDetectionPipelineState() -> MTLRenderPipelineState? {
        guard let vertexFunction = library.makeFunction(name: "obstacleDetectionVertex"),
              let fragmentFunction = library.makeFunction(name: "obstacleDetectionFragment") else {
                return nil
        }
        
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = vertexFunction
        descriptor.fragmentFunction = fragmentFunction
        //descriptor.depthAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        //descriptor.colorAttachments[0].pixelFormat = renderDestination.colorPixelFormat
        
        descriptor.depthAttachmentPixelFormat = sceneView.depthPixelFormat
        descriptor.colorAttachments[0].pixelFormat = sceneView.colorPixelFormat
        
        return try? device.makeRenderPipelineState(descriptor: descriptor)
    }
    
    //0220
    func makeBboxPipelineState() -> MTLRenderPipelineState? {
        
        guard let vertexFunction = library.makeFunction(name: "bboxVertex"),
              let fragmentFunction = library.makeFunction(name: "bboxFragment") else {
                return nil
        }
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = vertexFunction
        descriptor.fragmentFunction = fragmentFunction
        //.bgra8Unorm
        //0222
       // descriptor.depthAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        //descriptor.depthAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        //descriptor.colorAttachments[0].pixelFormat = renderDestination.colorPixelFormat
        descriptor.depthAttachmentPixelFormat = sceneView.depthPixelFormat
        descriptor.colorAttachments[0].pixelFormat = sceneView.colorPixelFormat
        return try? device.makeRenderPipelineState(descriptor: descriptor)
    }
    
    func makeUnprojectionPipelineState() -> MTLRenderPipelineState? {
        guard let vertexFunction = library.makeFunction(name: "unprojectVertex") else {
                return nil
        }
        
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = vertexFunction
        descriptor.isRasterizationEnabled = false
        //descriptor.depthAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        //descriptor.colorAttachments[0].pixelFormat = renderDestination.colorPixelFormat
        descriptor.depthAttachmentPixelFormat = sceneView.depthPixelFormat
        descriptor.colorAttachments[0].pixelFormat = sceneView.colorPixelFormat
        
        return try? device.makeRenderPipelineState(descriptor: descriptor)
    }
    
    func makeRGBPipelineState() -> MTLRenderPipelineState? {
        guard let vertexFunction = library.makeFunction(name: "rgbVertex"),
            let fragmentFunction = library.makeFunction(name: "rgbFragment") else {
                return nil
        }
        
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = vertexFunction
        descriptor.fragmentFunction = fragmentFunction
        //descriptor.depthAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        //descriptor.colorAttachments[0].pixelFormat = renderDestination.colorPixelFormat
        descriptor.depthAttachmentPixelFormat = sceneView.depthPixelFormat
        descriptor.colorAttachments[0].pixelFormat = sceneView.colorPixelFormat
        
        return try? device.makeRenderPipelineState(descriptor: descriptor)
    }
    
    func makeParticlePipelineState() -> MTLRenderPipelineState? {
        guard let vertexFunction = library.makeFunction(name: "particleVertex"),
            let fragmentFunction = library.makeFunction(name: "particleFragment") else {
                return nil
        }
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = vertexFunction
        descriptor.fragmentFunction = fragmentFunction
        //descriptor.depthAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        //descriptor.colorAttachments[0].pixelFormat = renderDestination.colorPixelFormat
        descriptor.depthAttachmentPixelFormat = sceneView.depthPixelFormat
        descriptor.colorAttachments[0].pixelFormat = sceneView.colorPixelFormat
        descriptor.colorAttachments[0].isBlendingEnabled = true
        descriptor.colorAttachments[0].sourceRGBBlendFactor = .sourceAlpha
        descriptor.colorAttachments[0].destinationRGBBlendFactor = .oneMinusSourceAlpha
        descriptor.colorAttachments[0].destinationAlphaBlendFactor = .oneMinusSourceAlpha
        
        return try? device.makeRenderPipelineState(descriptor: descriptor)
    }
    
    /// Makes sample points on camera image, also precompute the anchor point for animation
    func makeGridPoints() -> [Float2] {
        let gridArea = cameraResolution.x * cameraResolution.y
        //cameraResolution.x : 1920.0, cameraResolution.y : 1440.0, gridArea : 2764800.0
        let spacing = sqrt(gridArea / Float(numGridPoints))
        // spacing: 74.36128
        let deltaX = Int(round(cameraResolution.x / spacing))
        let deltaY = Int(round(cameraResolution.y / spacing))
        //deltaX : 28
        //deltaY : 21
        var points = [Float2]()
        for gridY in 0 ..< deltaY {
            let alternatingOffsetX = Float(gridY % 2) * spacing / 2
            for gridX in 0 ..< deltaX {
                let cameraPoint = Float2(alternatingOffsetX + (Float(gridX) + 0.5) * spacing, (Float(gridY) + 0.5) * spacing)
                points.append(cameraPoint)
            }
        }
        
        return points
    }
    
    func makeTextureCache() -> CVMetalTextureCache {
        // Create captured image texture cache
        var cache: CVMetalTextureCache!
        CVMetalTextureCacheCreate(nil, nil, device, nil, &cache)
        
        return cache
    }
    
    func makeTexture(fromPixelBuffer pixelBuffer: CVPixelBuffer, pixelFormat: MTLPixelFormat, planeIndex: Int) -> CVMetalTexture? {
        let width = CVPixelBufferGetWidthOfPlane(pixelBuffer, planeIndex)
        let height = CVPixelBufferGetHeightOfPlane(pixelBuffer, planeIndex)
        
        var texture: CVMetalTexture? = nil
        let status = CVMetalTextureCacheCreateTextureFromImage(nil, textureCache, pixelBuffer, nil, pixelFormat, width, height, planeIndex, &texture)
        
        if status != kCVReturnSuccess {
            texture = nil
        }

        return texture
    }
    
    static func cameraToDisplayRotation(orientation: UIInterfaceOrientation) -> Int {
        switch orientation {
        case .landscapeLeft:
            return 180
        case .portrait:
            return 90
        case .portraitUpsideDown:
            return -90
        default:
            return 0
        }
    }
    
    static func makeRotateToARCameraMatrix(orientation: UIInterfaceOrientation) -> matrix_float4x4 {
        // flip to ARKit Camera's coordinate
        let flipYZ = matrix_float4x4(
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1] )

        let rotationAngle = Float(cameraToDisplayRotation(orientation: orientation)) * .degreesToRadian
        return flipYZ * matrix_float4x4(simd_quaternion(rotationAngle, Float3(0, 0, 1)))
    }
    
    public func savePointsToFile() {
        
        // 1
        var fileToWrite = ""
        let headers = ["ply", "format ascii 1.0", "element vertex \(currentPointCount)", "property float x", "property float y", "property float z", "property uchar red", "property uchar green", "property uchar blue", "property uchar alpha", "element face 0", "property list uchar int vertex_indices", "end_header"]
        for header in headers {
            fileToWrite += header
            fileToWrite += "\r\n"
        }
        
        // 2
        for i in 0..<currentPointCount {
        
            // 3
            let point = particlesBuffer[i]
            let colors = point.color
        
            // 4
            let red = Int(colors.x * 255.0).clamped(to: 0...255)
            let green = Int(colors.y * 255.0).clamped(to: 0...255)
            let blue = Int(colors.z * 255.0).clamped(to: 0...255)
            
            // 5
            let pvValue = "\(point.position.x) \(point.position.y) \(point.position.z) \(Int(red)) \(Int(green)) \(Int(blue)) 255"
            fileToWrite += pvValue
            fileToWrite += "\r\n"
        }
        // 6
        let paths = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)
        let documentsDirectory = paths[0]
        let file = documentsDirectory.appendingPathComponent("ply_\(UUID().uuidString).ply")
        
        do {
            // 7
            try fileToWrite.write(to: file, atomically: true, encoding: String.Encoding.ascii)
        }
        catch {
            print("Failed to write PLY file", error)
        }
    }
}

extension Comparable {
    func clamped(to limits: ClosedRange<Self>) -> Self {
        return min(max(self, limits.lowerBound), limits.upperBound)
    }
}
