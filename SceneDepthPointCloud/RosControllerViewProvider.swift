//
//  RosControllerViewProvider.swift
//  SceneDepthPointCloud
//
//  Created by Yejin on 2023/01/31.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation
import UIKit
import OSLog
import ARKit

final class RosControllerViewProvider {
    private let logger = Logger(subsystem: "com.cherrrity.zero-lidar-logger", category: "RosControllerViewProvider")
    
    // Global/connection
    private let urlTextField = UITextField()
    private let urlTextFieldLabel = UILabel()
    private let masterSwitch = UISwitch()
    
    private struct PubEntry {
        var label: UILabel
        var labelText: String
        var topicNameField: UITextField?
        var defaultTopicName: String?
        var stateSwitch: UISwitch
        var rateStepper: UIStepper
        var rateStepperLabel: UILabel
        var rateMin: Double = 0.5
        var rateMax: Double = 30.0
        var rateDefault: Double = PubController.defaultRate
        var rateStep: Double = 0.5
    }
    
    private var pubEntries: [PubController.PubType: PubEntry]! = nil
    private var transformsEntry: PubEntry! = nil
    private var depthEntry: PubEntry! = nil
    private var pointCloudEntry: PubEntry! = nil
    private var cameraEntry: PubEntry! = nil
    private var locationEntry : PubEntry! = nil
    private var obstacleEntry : PubEntry! = nil
    
    private let session: ARSession
    private let pubController: PubController
    
    /// The provided view.
    public private(set) var view: UIView! = nil
    
    public init(pubController: PubController, session: ARSession) {
        self.logger.debug("init")
        
        self.pubController = pubController
        self.session = session
        
        // Pub UI entries
        self.obstacleEntry = self.createPubEntry(pubType: .obstacle, labelText: "Obstacle", defaultTopicName: "ipad/obstacle")
        
        self.pubEntries = [
            .obstacle: self.obstacleEntry
        ]
        
        self.pubEntries.forEach { (_: PubController.PubType, pubEntry: PubEntry) in
            self.initViewsFromEntry(pubEntry)
        }
        
        // WebSocket URL field, label, and global switch
        self.initViews(uiLabel: urlTextFieldLabel, labelText: "Remote bridge", uiTextField: urlTextField, uiStatusSwitch: masterSwitch, textFieldPlaceholder: "192.168.0.xyz:abcd")
        
        // Stack with all the ROS config
        // TODO add separator between IP address and pub controls
        let labelsStackView = self.createVerticalStack(arrangedSubviews: [urlTextFieldLabel, self.obstacleEntry.label])
        let textFieldsStackView = self.createVerticalStack(arrangedSubviews: [urlTextField, UIView(), self.obstacleEntry.topicNameField!])
        let statusSwitchesView = self.createVerticalStack(arrangedSubviews: [masterSwitch, self.obstacleEntry.stateSwitch])
        let steppersView = self.createVerticalStack(arrangedSubviews: [UIView(), self.obstacleEntry.rateStepper])
        let stepperDisplaysView = self.createVerticalStack(arrangedSubviews: [UIView(), self.obstacleEntry.rateStepperLabel])
        
        let rosStackView = UIStackView(arrangedSubviews: [labelsStackView, textFieldsStackView, statusSwitchesView, steppersView, stepperDisplaysView])
        rosStackView.translatesAutoresizingMaskIntoConstraints = false
        rosStackView.axis = .horizontal
        rosStackView.spacing = 10
        
        self.view = rosStackView
    }
    
    private func createPubEntry(pubType: PubController.PubType, labelText: String, defaultTopicName: String? = nil) -> PubEntry {
        let rateDefault = self.pubController.getPubRate(pubType) ?? PubController.defaultRate
        return PubEntry(label: UILabel(), labelText: labelText, topicNameField: nil != defaultTopicName ? UITextField() : nil, defaultTopicName: defaultTopicName, stateSwitch: UISwitch(), rateStepper: UIStepper(), rateStepperLabel: UILabel(), rateDefault: rateDefault)
    }
    
    private func initViewsFromEntry(_ pubEntry: PubEntry) {
        self.initViews(uiLabel: pubEntry.label, labelText: pubEntry.labelText, uiTextField: pubEntry.topicNameField, uiStatusSwitch: pubEntry.stateSwitch, textFieldPlaceholder: pubEntry.defaultTopicName, useAsDefaultText: true, pubEntry: pubEntry)
    }
    
    private func initViews(uiLabel: UILabel, labelText: String, uiTextField: UITextField?, uiStatusSwitch: UISwitch, textFieldPlaceholder: String?, useAsDefaultText: Bool = false, pubEntry: PubEntry? = nil) {
        print("test")
        if nil != uiTextField {
            uiTextField!.borderStyle = UITextField.BorderStyle.bezel
            uiTextField!.clearButtonMode = UITextField.ViewMode.whileEditing
            uiTextField!.autocorrectionType = UITextAutocorrectionType.no
            if nil != textFieldPlaceholder {
                uiTextField!.placeholder = textFieldPlaceholder
                if useAsDefaultText {
                    uiTextField!.text = textFieldPlaceholder
                }
            }
            uiTextField!.addTarget(self, action: #selector(textFieldValueChanged), for: .editingDidEndOnExit)
        }
        uiLabel.attributedText = NSAttributedString(string: labelText)
        uiStatusSwitch.preferredStyle = UISwitch.Style.checkbox
        uiStatusSwitch.addTarget(self, action: #selector(switchStatusChanged), for: .valueChanged)
        if nil != pubEntry {
            pubEntry!.rateStepper.autorepeat = true
            pubEntry!.rateStepper.isContinuous = true
            pubEntry!.rateStepper.minimumValue = pubEntry!.rateMin
            pubEntry!.rateStepper.maximumValue = pubEntry!.rateMax
            pubEntry!.rateStepper.stepValue = pubEntry!.rateStep
            pubEntry!.rateStepper.value = pubEntry!.rateDefault
            pubEntry!.rateStepper.isEnabled = false
            pubEntry!.rateStepper.addTarget(self, action: #selector(stepperValueChanged), for: .valueChanged)
            pubEntry!.rateStepperLabel.text = RosControllerViewProvider.rateAsString(pubEntry!.rateStepper.value)
        }
    }
    
    private func createVerticalStack(arrangedSubviews: [UIView]) -> UIStackView {
        let stackView = UIStackView(arrangedSubviews: arrangedSubviews)
        stackView.translatesAutoresizingMaskIntoConstraints = false
        stackView.axis = .vertical
        stackView.spacing = 10
        stackView.alignment = UIStackView.Alignment.fill
        stackView.distribution = UIStackView.Distribution.fillEqually
        return stackView
    }
    
    @objc
    private func textFieldValueChanged(view: UIView) {
        switch view {
        case self.urlTextField:
            self.updateUrl()
        case self.obstacleEntry.topicNameField:
            self.updatePubTopic(.obstacle)
        default:
            break
        }
    }
    
    @objc
    private func switchStatusChanged(view: UIView) {
        switch view {
        case self.masterSwitch:
            self.updateMasterSwitch()
        case self.obstacleEntry.stateSwitch:
            self.updateTopicState(.obstacle)
        default:
            break
        }
    }
    
     @objc
     private func stepperValueChanged(view: UIView) {
        switch view {
        case self.obstacleEntry.rateStepper:
            self.updateRate(.obstacle)
        default:
            break
        }
     }
    
    private func updateUrl() {
        // Enable pub controller and/or update URL
        if self.pubController.enable(url: self.urlTextField.text) {
            // It worked, so turn switch on
            self.masterSwitch.setOn(true, animated: true)
        } else {
            // It fails, so turn off switch and disable
            self.masterSwitch.setOn(false, animated: true)
            self.pubController.disable()
        }
    }
    
    private func updateMasterSwitch() {
        if self.masterSwitch.isOn {
            self.updateUrl()
        } else {
            // Disable pub controller
            self.pubController.disable()
        }
    }
    
    private func updatePubTopic(_ pubType: PubController.PubType) {
        let pubEntry = self.pubEntries[pubType]!
        if self.pubController.updatePubTopic(pubType: .depth, topicName: pubEntry.topicNameField?.text!) {
            pubEntry.stateSwitch.setOn(true, animated: true)
            self.updateTopicState(pubType)
        } else {
            // Disable pub and turn off switch
            self.pubController.disablePub(pubType: pubType)
            pubEntry.stateSwitch.setOn(false, animated: true)
        }
    }
    
    private func updateTopicState(_ pubType: PubController.PubType) {
        let pubEntry = self.pubEntries[pubType]!
        if pubEntry.stateSwitch.isOn {
            // Enable publishing
            if self.pubController.enablePub(pubType: pubType, topicName: pubEntry.topicNameField?.text!) {
                pubEntry.rateStepper.isEnabled = true
                // Enable master switch if not already enabled
                if !self.masterSwitch.isOn {
                    self.masterSwitch.setOn(true, animated: true)
                    self.updateMasterSwitch()
                }
            } else {
                // Enabling failed, so disable publishing & stepper and turn off switch
                pubEntry.stateSwitch.setOn(false, animated: true)
                pubEntry.rateStepper.isEnabled = false
                self.pubController.disablePub(pubType: pubType)
            }
        } else {
            // Disable publishing and stepper
            self.pubController.disablePub(pubType: pubType)
            pubEntry.rateStepper.isEnabled = false
        }
    }
    
    private func updateRate(_ pubType: PubController.PubType) {
        // Update display
        let pubEntry = self.pubEntries[pubType]!
        let rate = pubEntry.rateStepper.value
        pubEntry.rateStepperLabel.text = RosControllerViewProvider.rateAsString(rate)
        // Update pub rate
        self.pubController.updatePubRate(pubType: pubType, rate: rate)
    }
    
    private static func rateAsString(_ rate: Double) -> String {
        return String(format: "%.1f Hz", rate)
    }
}


