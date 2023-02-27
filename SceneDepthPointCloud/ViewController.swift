/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Main view controller for the AR experience.
*/

import UIKit
import Metal
import MetalKit
import ARKit
import CoreLocation
import Vision

struct Prediction {
    let labelIndex: Int
    let confidence: Float
    let boundingBox: CGRect
}

final class ViewController: UIViewController, ARSCNViewDelegate, ARSessionDelegate,  CLLocationManagerDelegate {

    @IBOutlet weak var debugTextView: UITextView!
    @IBOutlet weak var debugImageView: UIImageView!
    
    private let isUIEnabled = true
    private let confidenceControl = UISegmentedControl(items: ["Low", "Medium", "High"])
    private let rgbRadiusSlider = UISlider()
    private let rosConnectedBtn = UIButton()
    private var isRosConnected = false
    private var controlsButtonImages: (hide: UIImage, show: UIImage)?
    private let controlsButton = UIButton()
    private var isControlsViewEnabled = true
    
    private var session = ARSession()
    
    private var locationManager = CLLocationManager()
    private var renderer: Renderer!
    
    private var rosControllerViewProvider: RosControllerViewProvider!
    private var pubController: PubController!
    private var pubManger: PubManager!
    
    // COREML
    var bufferSize: CGSize = .zero
    var visionRequests = [VNRequest]()
    let dispatchQueueML = DispatchQueue(label: "com.cherrrity.coreML-with-lidar")
    var latestPrediction : String = "…"
    
    public func setPubManager(pubManager: PubManager) {
        self.pubController = pubManager.pubController
        self.session = pubManager.session
        self.locationManager = pubManager.locationManager
    }
    
    var model_name = "YOLOv3TinyInt8LUT"
    //YOLOv3TinyInt8LUT
    var rootLayer: CALayer! = nil
    var detectionOverlay: CALayer! = nil
    // Vision parts
    private var requests = [VNRequest]()
    
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        guard let device = MTLCreateSystemDefaultDevice() else {
            print("Metal is not supported on this device")
            return
        }
        
        session.delegate = self
        locationManager.delegate = self
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.requestWhenInUseAuthorization()
        locationManager.pausesLocationUpdatesAutomatically = false
        locationManager.startUpdatingLocation()
        locationManager.startUpdatingHeading()
        
        // Set the view to use the default device
        if let view = view as? MTKView {
            view.device = device
            
            view.backgroundColor = UIColor.clear
            // we need this to enable depth test
            view.autoResizeDrawable = true
            view.depthStencilPixelFormat = .depth32Float
            view.contentScaleFactor = 1
            view.delegate = self
            
            // Configure the renderer to draw to the view
            renderer = Renderer(session: session, metalDevice: device, renderDestination: view)
            renderer.drawRectResized(size: view.bounds.size)
        }
        
        self.debugImageView.layer.zPosition = 1
        
        // Confidence control
        confidenceControl.backgroundColor = .white
        confidenceControl.selectedSegmentIndex = renderer.confidenceThreshold
        confidenceControl.addTarget(self, action: #selector(viewValueChanged), for: .valueChanged)
        
        // RGB Radius control
        rgbRadiusSlider.minimumValue = 0
        rgbRadiusSlider.maximumValue = 1.5
        rgbRadiusSlider.isContinuous = true
        rgbRadiusSlider.value = renderer.rgbRadius
        rgbRadiusSlider.addTarget(self, action: #selector(viewValueChanged), for: .valueChanged)
        
        // ROS Connected Button
        let iconHide = UIImage(systemName: "gearshape", withConfiguration: UIImage.SymbolConfiguration(scale: .large))!.withTintColor(UIColor(white: 1.0, alpha: 0.5), renderingMode: .alwaysOriginal)
        let iconShow = UIImage(systemName: "gearshape.fill", withConfiguration: UIImage.SymbolConfiguration(scale: .large))!.withTintColor(UIColor(white: 1.0, alpha: 0.5), renderingMode: .alwaysOriginal)
        self.controlsButtonImages = (hide: iconHide, show: iconShow)
        self.controlsButton.setImage(self.controlsButtonImages!.hide, for: .normal)
        self.controlsButton.addTarget(self, action: #selector(controlsButtonPressed), for: .touchUpInside)
        self.controlsButton.translatesAutoresizingMaskIntoConstraints = false
        
        self.rosControllerViewProvider = RosControllerViewProvider(pubController: self.pubController!, session: self.session)
        
        let stackView = UIStackView(arrangedSubviews: [rosControllerViewProvider.view!, confidenceControl, rgbRadiusSlider])
        stackView.isHidden = !isUIEnabled
        stackView.translatesAutoresizingMaskIntoConstraints = false
        stackView.axis = .vertical
        stackView.spacing = 20
        
        view.addSubview(stackView)
        view.addSubview(controlsButton)
        
        NSLayoutConstraint.activate([
            stackView.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            stackView.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -50),
            self.controlsButton.bottomAnchor.constraint(equalTo: self.view.bottomAnchor, constant: -30),
            self.controlsButton.leftAnchor.constraint(equalTo: self.view.leftAnchor, constant: 30),
        ])
        
        // setup Vision parts
        
        bufferSize.width = view.bounds.width
        bufferSize.height = view.bounds.height
        
        detectionOverlay = CALayer() // container layer that has all the renderings of the observations
        detectionOverlay.name = "DetectionOverlay"
        detectionOverlay.bounds = CGRect(x: 0.0,
                                         y: 0.0,
                                         width: view.bounds.width,
                                         height: view.bounds.height)
        detectionOverlay.position = CGPoint(x: view.bounds.midX, y: view.bounds.midY)
        
        rootLayer = view.layer
        detectionOverlay.frame = rootLayer.bounds
        rootLayer.addSublayer(detectionOverlay)
        
        updateLayerGeometry()
        setupVision()
        
        loopCoreMLUpdate()
    }
    
    @discardableResult
    func setupVision() -> NSError? {
        // Setup Vision parts
        let error: NSError! = nil
        
        guard let modelURL = Bundle.main.url(forResource: model_name, withExtension: "mlmodelc") else {
            return NSError(domain: "ViewController", code: -1, userInfo: [NSLocalizedDescriptionKey: "Model file is missing"])
        }
        do {
            let selectedModel = try VNCoreMLModel(for: MLModel(contentsOf: modelURL))
            let classificationRequest = VNCoreMLRequest(model: selectedModel,  completionHandler: { (request, error) in
                DispatchQueue.main.async(execute: {
                    // perform all the UI updates on the main queue
                    if let results = request.results {
                        self.drawVisionRequestResults(results)
                        if let imageBuffer = self.session.currentFrame?.capturedImage {
                            //self.debugImageView.image = UIImage(ciImage: CIImage(cvPixelBuffer: imageBuffer))
                            self.debugImageView.image = UIImage(ciImage: CIImage(cvPixelBuffer: imageBuffer))

                        }
                    }
                })
            })
            classificationRequest.imageCropAndScaleOption = VNImageCropAndScaleOption.centerCrop
            
            self.visionRequests = [classificationRequest]
            //objectRecognition.imageCropAndScaleOption = VNImageCropAndScaleOption.centerCrop
            
        } catch let error as NSError {
            print("Model loading went wrong: \(error)")
        }

        return error
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Create a world-tracking configuration, and
        // enable the scene depth frame-semantic.
        let configuration = ARWorldTrackingConfiguration()
        configuration.frameSemantics = [.sceneDepth, .smoothedSceneDepth]
        
        let selectedVideoFormat = ARWorldTrackingConfiguration.supportedVideoFormats[1]
        configuration.videoFormat = selectedVideoFormat
        print(selectedVideoFormat) //imageResolution=(1920, 1080) framesPerSecond=(60) for iPhone 12 Pro

        // Run the view's session
        session.run(configuration)
        
        // The screen shouldn't dim during AR experiences.
        UIApplication.shared.isIdleTimerDisabled = true
    }
    
    @objc
    private func controlsButtonPressed() {
        // Just invert current state, update view and button
        self.isControlsViewEnabled = !self.isControlsViewEnabled
    }
    
    @objc
    private func viewValueChanged(view: UIView) {
        switch view {
            
        case confidenceControl:
            renderer.confidenceThreshold = confidenceControl.selectedSegmentIndex
            
        case rgbRadiusSlider:
            renderer.rgbRadius = rgbRadiusSlider.value
            
        default:
            break
        }
    }
    
    // Auto-hide the home indicator to maximize immersion in AR experiences.
    override var prefersHomeIndicatorAutoHidden: Bool {
        return true
    }
    
    // Hide the status bar to maximize immersion in AR experiences.
    override var prefersStatusBarHidden: Bool {
        return true
    }
    
    
    func session(_ session: ARSession, didUpdate frame: ARFrame, didFailWithError error: Error) {
        // Present an error message to the user.
        guard error is ARError else { return }
        let errorWithInfo = error as NSError
        let messages = [
            errorWithInfo.localizedDescription,
            errorWithInfo.localizedFailureReason,
            errorWithInfo.localizedRecoverySuggestion
        ]
        let errorMessage = messages.compactMap({ $0 }).joined(separator: "\n")
        DispatchQueue.main.async {
            // Present an alert informing about the error that has occurred.
            let alertController = UIAlertController(title: "The AR session failed.", message: errorMessage, preferredStyle: .alert)
            let restartAction = UIAlertAction(title: "Restart Session", style: .default) { _ in
                alertController.dismiss(animated: true, completion: nil)
                if let configuration = self.session.configuration {
                    self.session.run(configuration, options: .resetSceneReconstruction)
                }
            }
            alertController.addAction(restartAction)
            self.present(alertController, animated: true, completion: nil)
        }
    }
    
    func drawVisionRequestResults(_ results: [Any]) {
        CATransaction.begin()
        CATransaction.setValue(kCFBooleanTrue, forKey: kCATransactionDisableActions)
        detectionOverlay.sublayers = nil // remove all the old recognized objects
        for observation in results where observation is VNRecognizedObjectObservation {
            guard let objectObservation = observation as? VNRecognizedObjectObservation else {
                continue
            }
            // Select only the label with the highest confidence.
            let topLabelObservation = objectObservation.labels[0]
            let objectBounds = VNImageRectForNormalizedRect(objectObservation.boundingBox, Int(bufferSize.width), Int(bufferSize.height))
            
            let shapeLayer = self.createRoundedRectLayerWithBounds(objectBounds)
            
            let textLayer = self.createTextSubLayerInBounds(objectBounds,
                                                            identifier: topLabelObservation.identifier,
                                                            confidence: topLabelObservation.confidence)
            
            self.debugTextView.text = String(format: "\(topLabelObservation.identifier) - Confidence:  %.2f", topLabelObservation.confidence)
            
            shapeLayer.addSublayer(textLayer)
            
            //draw yolo bbox
            detectionOverlay.addSublayer(shapeLayer)
            
        }
        self.updateLayerGeometry()
        CATransaction.commit()
    }
    
    func classificationCompleteHandler(request: VNRequest, error: Error?) {
        // Catch Errors
        if error != nil {
            print("Error: " + (error?.localizedDescription)!)
            return
        }
        guard let results = request.results else {
            print("No results")
            return
        }
        
        if results.count <= 0 {
            return
        }
        
        var classifications = ""
        
        print(results[0].confidence)
        
        for case let foundObject as VNRecognizedObjectObservation in results {
            let bestLabel = foundObject.labels.first! // Label with highest confidence
            let objectBounds = foundObject.boundingBox // Normalized between [0,1]
            let confidence = foundObject.confidence // Confidence for the predicted class

            // Use the computed values.
            print(bestLabel.identifier, confidence, objectBounds)
            classifications.append("\(bestLabel.identifier) \(String(format:"- %.2f", confidence)) \n")
        }
        
        
        DispatchQueue.main.async {
            // Array to store final predictions (after post-processing)

            // Print Classifications
            print("--")
            
            // Display Debug Text on screen
            var debugText:String = ""
            debugText += classifications
            self.debugTextView.text = debugText
            
            // Store the latest prediction
            var objectName:String = "…"
            objectName = classifications.components(separatedBy: "-")[0]
            objectName = objectName.components(separatedBy: ",")[0]
            self.latestPrediction = objectName
            
        }
    }
    
    public func IoU(_ a: CGRect, _ b: CGRect) -> Float {
        let intersection = a.intersection(b)
        let union = a.union(b)
        return Float((intersection.width * intersection.height) / (union.width * union.height))
    }
    
    func updateLayerGeometry() {
        let bounds = rootLayer.bounds
        var scale: CGFloat
        
        
        let xScale: CGFloat = bounds.size.width / bufferSize.height
        let yScale: CGFloat = bounds.size.height / bufferSize.width
        //bounds.size.width : 1194.0
        //bufferSize.height : 834.0
        //bounds.size.height: 834.0
        //bufferSize.width : 1194.0
        //xScale : 1.4316546762589928
        //yScale : 0.6984924623115578
        
        scale = fmax(xScale, yScale)
        if scale.isInfinite {
            scale = 1.0
        }
        CATransaction.begin()
        CATransaction.setValue(kCFBooleanTrue, forKey: kCATransactionDisableActions)
        // rotate the layer into screen orientation and scale and mirror
        print("CGFloat(.pi / 3.0) : \(CGFloat(.pi / 3.0))")
        //CGFloat(.pi / 2.0) : 1.57
        //CGFloat(.pi / 3.0) : 1.04
        
        detectionOverlay.setAffineTransform(CGAffineTransform(rotationAngle: CGFloat(0.0)) .scaledBy(x: scale, y: -scale))
        // center the layer
        detectionOverlay.position = CGPoint(x: bounds.midX, y: bounds.midY)
        
        CATransaction.commit()
        
    }
    
    func createTextSubLayerInBounds(_ bounds: CGRect, identifier: String, confidence: VNConfidence) -> CATextLayer {
        let textLayer = CATextLayer()
        textLayer.name = "Object Label"
        let formattedString = NSMutableAttributedString(string: String(format: "\(identifier)- Confidence:  %.2f", confidence))
        let largeFont = UIFont(name: "Helvetica", size: 24.0)!
        formattedString.addAttributes([NSAttributedString.Key.font: largeFont], range: NSRange(location: 0, length: identifier.count))
        textLayer.string = formattedString
        textLayer.bounds = CGRect(x: 0, y: 0, width: bounds.size.height - 10, height: bounds.size.width - 10)
        textLayer.position = CGPoint(x: bounds.midX, y: bounds.midY)
        textLayer.shadowOpacity = 0.7
        textLayer.shadowOffset = CGSize(width: 2, height: 2)
        textLayer.foregroundColor = CGColor(colorSpace: CGColorSpaceCreateDeviceRGB(), components: [0.0, 0.0, 0.0, 1.0])
        textLayer.contentsScale = 2.0 // retina rendering
        // rotate the layer into screen orientation and scale and mirror
        textLayer.setAffineTransform(CGAffineTransform(rotationAngle: CGFloat(.pi / 2.0)).scaledBy(x: 1.0, y: -1.0))
        return textLayer
    }
    
    func createRoundedRectLayerWithBounds(_ bounds: CGRect) -> CALayer {
        let shapeLayer = CALayer()
        shapeLayer.bounds = bounds
        shapeLayer.position = CGPoint(x: bounds.midX, y: bounds.midY)
        shapeLayer.name = "Found Object"
        shapeLayer.backgroundColor = CGColor(colorSpace: CGColorSpaceCreateDeviceRGB(), components: [1.0, 1.0, 0.2, 0.4])
        return shapeLayer
    }
    
    
    func loopCoreMLUpdate() {
            // Continuously run CoreML whenever it's ready. (Preventing 'hiccups' in Frame Rate)
            
        dispatchQueueML.async {
            // 1. Run Update.
            self.updateCoreML()
            
            // 2. Loop this function.
            self.loopCoreMLUpdate()
        }
        
    }
    
    func updateCoreML() {
        ///////////////////////////
        // Get Camera Image as RGB
        let pixbuff : CVPixelBuffer? = (session.currentFrame?.capturedImage)
        if pixbuff == nil { return }
        let ciImage = CIImage(cvPixelBuffer: pixbuff!)
        

        // Note: Not entirely sure if the ciImage is being interpreted as RGB, but for now it works with the Inception model.
        // Note2: Also uncertain if the pixelBuffer should be rotated before handing off to Vision (VNImageRequestHandler) - regardless, for now, it still works well with the Inception model.
        
        ///////////////////////////
        // Prepare CoreML/Vision Request
       // let imageRequestHandler = VNSequenceRequestHandler()
      //  let imageRequestHandler = VNImageRequestHandler(ciImage: ciImage, orientation: .right, options: [:])
        let imageRequestHandler = VNImageRequestHandler(ciImage: ciImage, options: [:])
        // let imageRequestHandler = VNImageRequestHandler(cgImage: cgImage!, orientation: myOrientation, options: [:]) // Alternatively; we can convert the above to an RGB CGImage and use that. Also UIInterfaceOrientation can inform orientation values.
        
        ///////////////////////////
        // Run Image Request
        do {
         //   try imageRequestHandler.perform(self.visionRequests, on: ciImage)
            try imageRequestHandler.perform(self.visionRequests)
        } catch {
            print(error)
        }
    }
}

public func exifOrientationFromDeviceOrientation() -> UIInterfaceOrientation {
    let curDeviceOrientation = UIDevice.current.orientation
    let exifOrientation: UIInterfaceOrientation
    
    switch curDeviceOrientation {
    case UIDeviceOrientation.portraitUpsideDown:  // Device oriented vertically, home button on the top
        exifOrientation = .portrait
    case UIDeviceOrientation.landscapeLeft:       // Device oriented horizontally, home button on the right
        exifOrientation = .landscapeLeft
    case UIDeviceOrientation.landscapeRight:      // Device oriented horizontally, home button on the left
        exifOrientation = .landscapeRight
    case UIDeviceOrientation.portrait:            // Device oriented vertically, home button on the bottom
        exifOrientation = .portrait
    default:
        exifOrientation = .portrait
    }
    return exifOrientation
}

// MARK: - MTKViewDelegate

extension ViewController: MTKViewDelegate {
    
    // Called whenever view changes orientation or layout is changed
    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        renderer.drawRectResized(size: size)
    }
    
    // Called whenever the view needs to render
    func draw(in view: MTKView) {
        renderer.draw()
    }

}

// MARK: - RenderDestinationProvider

protocol RenderDestinationProvider {
    var currentRenderPassDescriptor: MTLRenderPassDescriptor? { get }
    var currentDrawable: CAMetalDrawable? { get }
    var colorPixelFormat: MTLPixelFormat { get set }
    var depthStencilPixelFormat: MTLPixelFormat { get set }
    var sampleCount: Int { get set }
}

extension MTKView: RenderDestinationProvider {
    
}
