import Foundation
import ARKit

class BoundingBox: SCNNode {
    
    static let extentChangedNotification = Notification.Name("BoundingBoxExtentChanged")
    static let positionChangedNotification = Notification.Name("BoundingBoxPositionChanged")
    static let scanPercentageChangedNotification = Notification.Name("ScanPercentageChanged")
    static let scanPercentageUserInfoKey = "ScanPercentage"
    static let boxExtentUserInfoKey = "BoxExtent"
    
    var extent = SIMD3<Float>(0.1, 0.1, 0.1) {
        didSet {
            extent = max(extent, minSize)
            updateVisualization()
            NotificationCenter.default.post(name: BoundingBox.extentChangedNotification,
                                            object: self)
        }
    }
    
    override var simdPosition: SIMD3<Float> {
        willSet(newValue) {
            if distance(newValue, simdPosition) > 0.001 {
                NotificationCenter.default.post(name: BoundingBox.positionChangedNotification,
                                                object: self)
            }
        }
    }
    
    var hasBeenAdjustedByUser = false
    private var maxDistanceToFocusPoint: Float = 0.1
    private var maxOutliarDistance: Float = 0.03
    
    private var minSize: Float = 0.01
    
    private var wireframe: Wireframe?
    
    private var sidesNode = SCNNode()
    private var sides: [BoundingBoxSide.Position: BoundingBoxSide] = [:]
    
    private var color = UIColor(red:159/255, green: 203/255, blue: 109/255, alpha: 1)
    
    private var cameraRaysAndHitLocations: [(ray: Ray, hitLocation: SIMD3<Float>)] = []
    private var frameCounter: Int = 0
    
    var progressPercentage: Int = 0
    private var isUpdatingCapturingProgress = false
    
    private var sceneView: ARSCNView
    
    internal var isSnappedToHorizontalPlane = false
    
    init(_ sceneView: ARSCNView) {
        self.sceneView = sceneView
        super.init()
        
        updateVisualization()
    }
    
    func fitOverPointCloud(_ pointCloud: [SIMD3<Float>], focusPoint: SIMD3<Float>?) {
        var filteredPoints: [SIMD3<Float>] = []
        
        for point in pointCloud {
            if let focus = focusPoint {
                // Skip this point if it is more than maxDistanceToFocusPoint meters away from the focus point.
                let distanceToFocusPoint = length(point - focus)
                //print(distanceToFocusPoint)
                if distanceToFocusPoint > maxDistanceToFocusPoint {
                    continue
                }
            }
            
            // Skip this point if it is an outlier (not at least 3 other points closer than 3 cm)
            var nearbyPoints = 0
            for otherPoint in pointCloud {
                if distance(point, otherPoint) < maxOutliarDistance {
                    nearbyPoints += 1
                    if nearbyPoints >= 3 {
                        filteredPoints.append(point)
                        break
                    }
                }
            }
        }
        
        guard !filteredPoints.isEmpty else { return }
        
        var localMin = -extent / 2
        var localMax = extent / 2
        
        print("TEST : \(filteredPoints.count)")
        
        for point in filteredPoints {
            //print(point)
            // The bounding box is in local coordinates, so convert point to local, too.
            let localPoint = self.simdConvertPosition(point, from: nil)
            
            localMin = min(localMin, localPoint)
            localMax = max(localMax, localPoint)
        }
        
        print(localMin)
        print(localMax)
        print("**************")
        // Update the position & extent of the bounding box based on the new min & max values.
        self.simdPosition += (localMax + localMin) / 2
        self.extent = localMax - localMin
    
        
        print("Extent : \(self.extent)")
    }
    
    func updateVisualization() {
        self.updateWireframe()
    }
    
    private func updateWireframe() {
        // When this method is called the first time, create the wireframe and add them as child node.
        guard let wireframe = self.wireframe else {
            let wireframe = Wireframe(extent: self.extent, color: UIColor(ciColor: CIColor(red: 102, green: 49, blue: 255)))
            self.addChildNode(wireframe)
            self.wireframe = wireframe
            return
        }
        
        // Otherwise just update the wireframe's size and position.
        wireframe.update(extent: self.extent)
    }

    func hideExtensionsOnAllAxes() {
        sides.forEach {
            $0.value.hideXAxisExtensions()
            $0.value.hideYAxisExtensions()
            $0.value.hideZAxisExtensions()
        }
    }
    
    func isHit(screenPos: CGPoint) -> Bool {
        let hitResults = sceneView.hitTest(screenPos, options: [
            .rootNode: sidesNode,
            .ignoreHiddenNodes: false])
        
        for result in hitResults where (result.node.parent as? BoundingBoxSide) != nil {
            return true
        }
        return false
    }
  
    /// Returns true if the given location differs from all hit locations in the cameraRaysAndHitLocations array
    /// by at least the threshold distance.
    func isHitLocationDifferentFromPreviousRayHitTests(_ location: SIMD3<Float>) -> Bool {
        let distThreshold: Float = 0.03
        for hitTest in cameraRaysAndHitLocations.reversed() {
            if distance(hitTest.hitLocation, location) < distThreshold {
                return false
            }
        }
        return true
    }
  
    private func sidesForAxis(_ axis: Axis) -> [BoundingBoxSide?] {
        switch axis {
        case .x:
            return [sides[.left], sides[.right]]
        case .y:
            return [sides[.top], sides[.bottom]]
        case .z:
            return [sides[.front], sides[.back]]
        }
    }
    
    func updateOnEveryFrame() {
        if let frame = sceneView.session.currentFrame {
            // Check if the bounding box should align its bottom with a nearby plane.
            tryToAlignWithPlanes(frame.anchors)
        }
        
        sides.forEach { $0.value.updateVisualizationIfNeeded() }
    }
    
    func tryToAlignWithPlanes(_ anchors: [ARAnchor]) {
        //guard !hasBeenAdjustedByUser, ViewController.instance?.scan?.state == 2 else { return }
        
        let bottomCenter = SIMD3<Float>(simdPosition.x, simdPosition.y - extent.y / 2, simdPosition.z)

        var distanceToNearestPlane = Float.greatestFiniteMagnitude
        var offsetToNearestPlaneOnY: Float = 0
        var planeFound = false
        
        // Check which plane is nearest to the bounding box.
        for anchor in anchors {
            guard let plane = anchor as? ARPlaneAnchor else {
                continue
            }
            guard let planeNode = sceneView.node(for: plane) else {
                continue
            }
            
            // Get the position of the bottom center of this bounding box in the plane's coordinate system.
            let bottomCenterInPlaneCoords = planeNode.simdConvertPosition(bottomCenter, from: parent)
            
            // Add 10% tolerance to the corners of the plane.
            let tolerance: Float = 0.1
            let minX = plane.center.x - plane.extent.x / 2 - plane.extent.x * tolerance
            let maxX = plane.center.x + plane.extent.x / 2 + plane.extent.x * tolerance
            let minZ = plane.center.z - plane.extent.z / 2 - plane.extent.z * tolerance
            let maxZ = plane.center.z + plane.extent.z / 2 + plane.extent.z * tolerance
            
            print(minX, maxX, minZ, maxZ)
            
            guard (minX...maxX).contains(bottomCenterInPlaneCoords.x) && (minZ...maxZ).contains(bottomCenterInPlaneCoords.z) else {
                continue
            }
            
            let offsetToPlaneOnY = bottomCenterInPlaneCoords.y
            let distanceToPlane = abs(offsetToPlaneOnY)
            
            if distanceToPlane < distanceToNearestPlane {
                distanceToNearestPlane = distanceToPlane
                offsetToNearestPlaneOnY = offsetToPlaneOnY
                planeFound = true
            }
        }
        
        guard planeFound else { return }
        
        // Check that the object is not already on the nearest plane (closer than 1 mm).
        let epsilon: Float = 0.001
        guard distanceToNearestPlane > epsilon else { return }
        
        // Check if the nearest plane is close enough to the bounding box to "snap" to that
        // plane. The threshold is half of the bounding box extent on the y axis.
        let maxDistance = extent.y / 2
        if distanceToNearestPlane < maxDistance && offsetToNearestPlaneOnY > 0 {
            // Adjust the bounding box position & extent such that the bottom of the box
            // aligns with the plane.
            simdPosition.y -= offsetToNearestPlaneOnY / 2
            extent.y += offsetToNearestPlaneOnY
        }
    }
    
    func contains(_ pointInWorld: SIMD3<Float>) -> Bool {
        let localMin = -extent / 2
        let localMax = extent / 2
        
        // The bounding box is in local coordinates, so convert point to local, too.
        let localPoint = self.simdConvertPosition(pointInWorld, from: nil)
        
        return (localMin.x...localMax.x).contains(localPoint.x) &&
            (localMin.y...localMax.y).contains(localPoint.y) &&
            (localMin.z...localMax.z).contains(localPoint.z)
    }
    
    required init?(coder aDecoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
