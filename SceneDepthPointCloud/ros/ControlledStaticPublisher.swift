import Foundation
import OSLog

/// Managed publisher that can be enabled/idsabled and for which we can't change the topic.
final class ControlledStaticPublisher: ControlledPublisher {
    private var logger = Logger(subsystem: "com.cherrrity.zero-lidar-logger", category: "ControlledStaticPublisher")
    
    private let topicName: String
    
    public init(interface: RosInterface, type: Any, topicName: String) {
        self.topicName = topicName
        super.init(interface: interface, type: type)
    }
    
    public override func enable(topicName: String? = nil) -> Bool {
        return super.enable(topicName: self.topicName)
    }
    
    public override func updateTopic(topicName: String? = nil) -> Bool {
        return super.updateTopic(topicName: self.topicName)
    }
}

