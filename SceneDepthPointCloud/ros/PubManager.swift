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
    
    private var pubObstacle : ControlledPublisher
    
    var centroids = [vector_float3]()
    
    
    public init() {
        /// Create controlled pub objects for all publishers

        self.pubObstacle = ControlledPublisher(interface: self.interface, type: sensor_msgs__Obstacle.self)
        
        let controlledPubs: [PubController.PubType: [ControlledPublisher]] = [
            .obstacle : [self.pubObstacle],
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
        self.startPubThread(id: "obstacle", pubType: .obstacle, publishFunc: self.publishObstacle)
    }

    private func publishObstacle(){
        
        let width = self.pubController.width
        let depth = self.pubController.depth
        let time = self.pubController.timestamp

        self.pubObstacle.publish(RosMessagesUtils.obstacleInfoToMsgs(time: time, width: width, depth: depth))
    }
}
