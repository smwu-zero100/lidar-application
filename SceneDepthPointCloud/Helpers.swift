/*
See LICENSE folder for this sample’s licensing information.

Abstract:
General Helper methods and properties
*/

import Foundation
import ARKit

typealias Float2 = SIMD2<Float>
typealias Float3 = SIMD3<Float>

extension Float {
    static let degreesToRadian = Float.pi / 180
}

extension matrix_float3x3 {
    mutating func copy(from affine: CGAffineTransform) {
        columns.0 = Float3(Float(affine.a), Float(affine.c), Float(affine.tx))
        columns.1 = Float3(Float(affine.b), Float(affine.d), Float(affine.ty))
        columns.2 = Float3(0, 0, 1)
    }
}

enum Axis {
    case x
    case y
    case z
    
    var normal: SIMD3<Float> {
        switch self {
        case .x:
            return SIMD3<Float>(1, 0, 0)
        case .y:
            return SIMD3<Float>(0, 1, 0)
        case .z:
            return SIMD3<Float>(0, 0, 1)
        }
    }
}


extension SCNMaterial {
    
    static func material(withDiffuse diffuse: Any?, respondsToLighting: Bool = false, isDoubleSided: Bool = true) -> SCNMaterial {
        let material = SCNMaterial()
        material.diffuse.contents = diffuse
        material.isDoubleSided = isDoubleSided
        if respondsToLighting {
            material.locksAmbientWithDiffuse = true
        } else {
            material.locksAmbientWithDiffuse = false
            material.ambient.contents = UIColor.black
            material.lightingModel = .constant
        }
        return material
    }
}

struct Ray {
    let origin: SIMD3<Float>
    let direction: SIMD3<Float>
    
    init(origin: SIMD3<Float>, direction: SIMD3<Float>) {
        self.origin = origin
        self.direction = direction
    }
    
    init(normalFrom pointOfView: SCNNode, length: Float) {
        let cameraNormal = normalize(pointOfView.simdWorldFront) * length
        self.init(origin: pointOfView.simdWorldPosition, direction: cameraNormal)
    }
}

extension simd_quatf {
    init(angle: Float, axis: Axis) {
        self.init(angle: angle, axis: axis.normal)
    }
}

extension float4x4 {
    var position: SIMD3<Float> {
        return columns.3.xyz
    }
}

extension SIMD4 where Scalar == Float {
    var xyz: SIMD3<Float> {
        return SIMD3<Float>(x, y, z)
    }

    init(_ xyz: SIMD3<Float>, _ w: Float) {
        self.init(xyz.x, xyz.y, xyz.z, w)
    }
}
