//
//  PubManager.swift
//  SceneDepthPointCloud
//
//  Created by Yejin on 2023/01/31.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation
import ARKit
import CoreLocation
import OSLog
/// Class that manages and instigates the publishing.
///
/// The actual work is done in a background thread.
final class PubManager {
    private let logger = Logger(subsystem: "com.christophebedard.lidar2ros", category: "PubManager")
    
    public let session = ARSession()
    public let pubController: PubController
    public let locationManager = CLLocationManager()
    private let interface = RosInterface()
    
    private var pubTf: ControlledStaticPublisher
    // private var pubTfStatic: ControlledStaticPublisher
    private var pubDepth: ControlledPublisher
    private var pubPointCloud: ControlledPublisher
    private var pubCamera: ControlledPublisher
    private var pubLocation : ControlledPublisher
    
    var centroids = [vector_float3]()
    
    public init() {
        /// Create controlled pub objects for all publishers
        // hello boin
        self.pubTf = ControlledStaticPublisher(interface: self.interface, type: tf2_msgs__TFMessage.self, topicName: "/tf")
        // FIXME: using /tf only for now because /tf_static does not seem to work
        // self.pubTfStatic = ControlledStaticPublisher(interface: self.interface, type: tf2_msgs__TFMessage.self, topicName: "/tf")
        self.pubDepth = ControlledPublisher(interface: self.interface, type: sensor_msgs__Image.self)
        self.pubPointCloud = ControlledPublisher(interface: self.interface, type: sensor_msgs__PointCloud2.self)
        self.pubCamera = ControlledPublisher(interface: self.interface, type: sensor_msgs__Image.self)
        self.pubLocation = ControlledPublisher(interface: self.interface, type: sensor_msgs__NavSatFix.self)
        
        let controlledPubs: [PubController.PubType: [ControlledPublisher]] = [
            //.transforms: [self.pubTf, self.pubTfStatic],
            .transforms: [self.pubTf],
            .depth: [self.pubDepth],
            .pointCloud: [self.pubPointCloud],
            .camera: [self.pubCamera],
            .location : [self.pubLocation]
        ]
        self.pubController = PubController(pubs: controlledPubs, interface: self.interface)
    }
    
    private func startPubThread(id: String, pubType: PubController.PubType, publishFunc: @escaping () -> Void) {
        self.logger.debug("start pub thread: \(id)")
        DispatchQueue.global(qos: .background).async {
            Thread.current.name = "PubManager: \(id)"
            var last = Date().timeIntervalSince1970
            while true {
                if !self.pubController.isEnabled {
                    continue
                }
                let interval = 1.0 / self.pubController.getPubRate(pubType)!
                // TODO find a better way: seems like busy sleep is the
                // most reliable way to do this but it wastes CPU time
                var now = Date().timeIntervalSince1970
                while now - last < interval {
                    now = Date().timeIntervalSince1970
                }
                last = Date().timeIntervalSince1970
                publishFunc()
            }
        }
    }
    
    /// Start managed publishing.
    public func start() {
        self.logger.debug("start")
        self.startPubThread(id: "tf", pubType: .transforms, publishFunc: self.publishTf)
        self.startPubThread(id: "depth", pubType: .depth, publishFunc: self.publishDepth)
        self.startPubThread(id: "pointcloud", pubType: .pointCloud, publishFunc: self.publishPointCloud)
        // TODO fix/implement
        // self.startPubThread(id: "camera", pubType: .camera, publishFunc: self.publishCamera)
        
       // self.startPubThread(id: "location", pubType: .location, publishFunc: self.publishLocation)
    }
    
    private func publishTf() {
        guard let currentFrame = self.session.currentFrame else {
            return
        }
        let timestamp = currentFrame.timestamp
        let cameraTf = currentFrame.camera.transform
        // TODO revert when /tf_static works
        // self.pubTf.publish(RosMessagesUtils.tfToTfMsg(time: time, tf: cameraTf))
        // self.pubTfStatic.publish(RosMessagesUtils.getTfStaticMsg(time: time))
        var tfMsg = RosMessagesUtils.tfToTfMsg(time: timestamp, tf: cameraTf)
        let tfStaticMsg = RosMessagesUtils.getTfStaticMsg(time: timestamp)
        tfMsg.transforms.append(contentsOf: tfStaticMsg.transforms)
        self.pubTf.publish(tfMsg)
    }
    
    private func publishDepth() {
        guard let currentFrame = self.session.currentFrame,
              let depthMap = currentFrame.sceneDepth?.depthMap else {
                return
        }
        let timestamp = currentFrame.timestamp
        self.pubDepth.publish(RosMessagesUtils.depthMapToImage(time: timestamp, depthMap: depthMap))
    }
    
    private func publishPointCloud() {
        guard let currentFrame = self.session.currentFrame,
              var pointCloud = currentFrame.rawFeaturePoints?.points else {
                return
        }
        // clustering code 예정
        
        centroids = initAndClustering(points: pointCloud)
        
        centroids.forEach { (point) in
            pointCloud.append(point)
        }
        
        let timestamp = currentFrame.timestamp
        self.pubPointCloud.publish(RosMessagesUtils.pointsToPointCloud2(time: timestamp, points: pointCloud))
    }

    private func publishLocation(){

        // location 데이터 받아오는지 잘 모르겠음!!!!!!!!!

        guard let currentlocation = self.locationManager.location,
              let currentFrame = self.session.currentFrame else{
            return
        }
        let locations = [currentlocation.coordinate.latitude, currentlocation.coordinate.longitude]
        let timestamp = currentFrame.timestamp
        
        self.pubLocation.publish(RosMessagesUtils.locationToNavsatFix(time: timestamp, location: locations))
    }
}

   
//    private func publishCamera() {
//        guard let currentFrame = self.session.currentFrame else {
//            return
//        }
//        let timestamp = currentFrame.timestamp
//        let cameraImage = currentFrame.capturedImage
//        self.pubCamera.publish(RosMessagesUtils.pixelBufferToImage(time: time, pixelBuffer: cameraImage))
//    }

