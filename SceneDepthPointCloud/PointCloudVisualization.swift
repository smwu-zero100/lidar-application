import SceneKit

protocol PointCloud {
    func createVisualization(for points: [SIMD3<Float>], color: UIColor, size: CGFloat) -> SCNGeometry?
}
extension PointCloud {
    func createVisualization(for points: [SIMD3<Float>], color: UIColor, size: CGFloat) -> SCNGeometry? {
        guard !points.isEmpty else { return nil }
        
        let stride = MemoryLayout<SIMD3<Float>>.size
        let pointData = Data(bytes: points, count: stride * points.count)
        
        // Create geometry source
        let source = SCNGeometrySource(data: pointData,
                                       semantic: SCNGeometrySource.Semantic.vertex,
                                       vectorCount: points.count,
                                       usesFloatComponents: true,
                                       componentsPerVector: 3,
                                       bytesPerComponent: MemoryLayout<Float>.size,
                                       dataOffset: 0,
                                       dataStride: stride)
        
        // Create geometry element
        let element = SCNGeometryElement(data: nil, primitiveType: .point, primitiveCount: points.count, bytesPerIndex: 0)
        element.pointSize = 0.001
        element.minimumPointScreenSpaceRadius = size
        element.maximumPointScreenSpaceRadius = size
        
        let pointsGeometry = SCNGeometry(sources: [source], elements: [element])
        pointsGeometry.materials = [SCNMaterial.material(withDiffuse: color)]
        
        return pointsGeometry
    }
}
