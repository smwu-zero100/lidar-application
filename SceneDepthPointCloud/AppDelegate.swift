import UIKit
import ARKit

@UIApplicationMain
class AppDelegate: UIResponder, UIApplicationDelegate {

    var window: UIWindow?
    private var pubManager: PubManager!

    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?) -> Bool {
        if !ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            // Ensure that the device supports scene depth and present
            //  an error-message view controller, if not.
            let storyboard = UIStoryboard(name: "Main", bundle: nil)
            window?.rootViewController = storyboard.instantiateViewController(withIdentifier: "unsupportedDeviceMessage")
        }else
        {
            self.pubManager = PubManager()
            (window?.rootViewController as! ViewController).setPubManager(pubManager: self.pubManager)
            self.pubManager.start()
        }
        return true
    }
}
