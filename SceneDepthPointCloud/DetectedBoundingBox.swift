import Foundation
import ARKit

class DetectedBoundingBox: SCNNode {
    
    init(points: [SIMD3<Float>], scale: CGFloat, color: UIColor = UIColor(displayP3Red: 1.0, green: 0, blue: 0, alpha: 1)) {
        super.init()
        
        var localMin = SIMD3<Float>(repeating: Float.greatestFiniteMagnitude)
        var localMax = SIMD3<Float>(repeating: -Float.greatestFiniteMagnitude)
        
        for point in points {
            localMin = min(localMin, point)
            localMax = max(localMax, point)
        }
        
        self.simdPosition += (localMax + localMin) / 2
        let extent = localMax - localMin
        let wireframe = Wireframe(extent: extent, color: color, scale: scale)
        self.addChildNode(wireframe)
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
