//
//  Kmeans.swift
//  SceneDepthPointCloud
//
//  Created by Yejin on 2023/02/01.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation
import ARKit

struct Vertex {
        var x: Float
        var y: Float
        var z: Float
    
        var centroidIndex: Int = -1
        func distanceTo(_ vertex: Vertex) -> Float {
            return sqrt((x - vertex.x) * (x - vertex.x) + (y - vertex.y) * (y - vertex.y) + (z - vertex.z) * (z - vertex.z))
        }
        static func centroid(of vertexs: [Vertex]) -> Vertex {
            var sumX:Float = 0.0
            var sumY:Float = 0.0
            var sumZ:Float = 0.0
            vertexs.forEach { v in
                sumX += v.x
                sumY += v.y
                sumZ += v.z
            }
            let centerX = Float(sumX / Float(vertexs.count))
            let centerY = Float(sumY / Float(vertexs.count))
            let centerZ = Float(sumZ / Float(vertexs.count))
            return Vertex(x: centerX, y: centerY, z: centerZ)
        }
    }

let K_COUNT = 3
let DATA_COUNT = 50
var k: [[Vertex]] = [[]]
var centroids = [Vertex]()
var rt_centroid = [vector_float3]()
var datas = [Vertex]()

func initAndClustering(points: MetalBuffer<ParticleUniforms>) -> [vector_float3] {
    //
        datas.removeAll()
        k.removeAll()
        centroids.removeAll()
        rt_centroid.removeAll()
    
        for i in 0..<points.count {
            let point = points[i]
            datas.append(Vertex(x: point.position.x, y: point.position.y, z: point.position.z))
        }
    
    //    points.forEach { (point) in
     //       datas.append(Vertex(x: point.x, y: point.y, z: point.z))
     //   }
        
        if points.count > 10  {
            (0..<K_COUNT).forEach { i in
                centroids.append(datas[i])
            }
        }
        var flag = true
        while flag {
            flag = false
            var temp = [[Vertex]](repeating: [], count: K_COUNT)
            for i in (0..<datas.count) {
                var minDistance = Float.greatestFiniteMagnitude
                var indexOfNearest = -1
                for (index, centroid) in centroids.enumerated() {
                    let distance = datas[i].distanceTo(centroid)
                    if distance < minDistance {
                        minDistance = distance
                        indexOfNearest = index
                    }
                }
                guard indexOfNearest != -1 else { continue }
                if datas[i].centroidIndex != indexOfNearest {
                    flag = true
                }
                datas[i].centroidIndex = indexOfNearest
                
                temp[indexOfNearest].append(datas[i])
            }
            
            centroids.removeAll()
            temp.forEach({ vertexs in
                let centroid = Vertex.centroid(of: vertexs)
                centroids.append(centroid)
                // print("Centeroid point info :: -> ", centroid.x, centroid.y, centroid.z)
            })
            // print("calculating")
        }
        print("centroid : \(centroids.count)")
        centroids.forEach { (point) in
            rt_centroid.append(vector_float3(x: point.x, y: point.y, z: point.z))
        }
        // print("end")
    
        return rt_centroid
        
    }
