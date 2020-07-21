//
//  ViewController.swift
//  AprilTagDetector
//
//  Created by djconnolly27 on 7/11/18.
//  Copyright © 2020 OccamLab. All rights reserved.
//

import UIKit
import ARKit
import FirebaseDatabase
import FirebaseStorage

class LocationData {
    var node: SCNNode
    var picture: UIImage
    var textNode: SCNNode
    var poseId: Int
    
    init(node: SCNNode, picture: UIImage, textNode: SCNNode, poseId: Int) {
        self.node = node
        self.picture = picture
        self.textNode = textNode
        self.poseId = poseId
    }
}

class ViewController: UIViewController, writeValueBackDelegate, writeNodeBackDelegate {
    
    //MARK: Properties

    @IBOutlet var sceneView: ARSCNView!
    // button to move to the other view
    @IBOutlet var moveToButton: UIButton!
    // label to print needed messages
    @IBOutlet var explainLabel: UILabel!
    
    var broadcastTags: UDPBroadcastConnection!
    var broadcastPoseConnection: UDPBroadcastConnection!
    var broadcastImagesConnection: UDPBroadcastConnection!
    
    var timer = Timer()
    
    var isProcessingFrame = false
    let f = imageToData()
    let aprilTagQueue = DispatchQueue(label: "edu.occamlab.apriltagfinder", qos: DispatchQoS.userInitiated)
    var tagData:[[Any]] = []
    var poseData:[[Any]] = []
    var locationData:[[Any]] = []
    var poseId: Int = 0
    
    var firebaseRef: DatabaseReference!
    var firebaseStorage: Storage!
    var firebaseStorageRef: StorageReference!
    
    //boolean to check whether the camera found tag or not
    var foundTag: Bool = false
    var isMovingBox: Bool = false
    var recordCurrentLocation: Bool = false
    
    // will save current box object in control
    var currentBoxNode: SCNNode = SCNNode()
    var currentTextNode: SCNNode = SCNNode()
    
    var nodeList: [LocationData] = []
    var currentFrameTransform: simd_float4x4 = simd_float4x4.init()
    
    /// Begin an ARSession when the app first loads
    override func viewDidLoad() {
        super.viewDidLoad()
        startSession()
        initFirebase()
        let tap: UITapGestureRecognizer = UITapGestureRecognizer(target: self, action: #selector(dismissKeyboard))
        view.addGestureRecognizer(tap)
    }
    
    /// Resign keyboard when screen is tapped
    @objc func dismissKeyboard() {
        view.endEditing(true)
    }
    
    /// Initialize the ARSession
    func startSession() {
        let configuration = ARWorldTrackingConfiguration()
        sceneView.session.run(configuration)
        //sceneView.debugOptions = ARSCNDebugOptions.showWorldOrigin
    }
    
    /// Initialize firebase reference
    func initFirebase() {
        firebaseRef = Database.database().reference()
        firebaseStorage = Storage.storage()
        firebaseStorageRef = firebaseStorage.reference()
    }
    
    /// Send selected types of data to ROS when button is pressed
    //
    @IBAction func startButtonTapped(_ sender: UIButton) {
        if sender.currentTitle == "Start Recording" {
            sender.setTitle("Stop Recording", for: .normal)
            clearData()
            explainLabel.text = "Started recording!"
            
            timer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(self.recordData), userInfo: nil, repeats: true)
        }
        else {
            timer.invalidate()
            timer = Timer()
            // delay to avoid pressing the button again before timer ends
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.2, execute: {
                self.popUpSaveView()
                self.clearData()
                sender.setTitle("Start Recording", for: .normal)
            })
        }

    }
    /// Move to the save location screen
    @IBAction func moveButtonTapped(_ sender: UIButton) {
        if foundTag == true {
            if sender.currentTitle == "Save Location" {
                sender.setTitle("Set Location", for: .normal)
                self.performSegue(withIdentifier: "SetLocation", sender: self)
            } else if isMovingBox == true {
                isMovingBox = false
                recordCurrentLocation = true
                sender.setTitle("Save Location", for: .normal)
            }
        } else {
            explainLabel.text = "You first have to find a tag"
        }
    }
    
    //move to manage location view when tapped
    @IBAction func manageButtonTapped(_ sender: Any) {
        self.performSegue(withIdentifier: "LocationInfo", sender: self)
    }
    
    // when moving to the other view, prepare delegate to get the data back
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        if segue.identifier == "SetLocation" {
            let saveLocationController = segue.destination as! SaveLocationController
            saveLocationController.delegate = self
        } else if segue.identifier == "LocationInfo" {
            let manageLocationController = segue.destination as! ManageLocationController
            manageLocationController.nodeList = nodeList
            manageLocationController.delegate = self
        }
    }
    
    // function called when dismissing the save location view
    func writeValueBack(value: [String:String]) {
        if let tempString = value["LocationName"] {
            explainLabel.text = tempString
            addBox(locationName: tempString)
        }
    }
    
    // function called when dismissing the manage location view
    // updates the nodes according to how the user editted them
    func writeNodeBack(nodes: [LocationData], deleteNodes: [LocationData]) {
        nodeList = nodes
        for x in deleteNodes {
            x.node.removeFromParentNode()
            x.textNode.removeFromParentNode()
            for (index, element) in locationData.enumerated() {
                if element[17] as! Int == x.poseId {
                    locationData.remove(at: index)
                }
            }
        }
    }
    
    // add a box to the scene with the location name.
    // created 0.6m in front of the current camera view
    func addBox(locationName: String) {
        let box = SCNBox(width: 0.05, height: 0.2, length: 0.05, chamferRadius: 0)
        
        let text = SCNText(string: locationName, extrusionDepth: 0)
        
        let cameraNode = sceneView.pointOfView
        let boxNode = SCNNode()
        let textNode = SCNNode()
        boxNode.geometry = box
        boxNode.name = locationName
        textNode.geometry = text
        textNode.name = locationName + "Text"
        let boxPosition = SCNVector3(0,0,-0.6)
        let textPosition = SCNVector3(0,0.1,0)
        
        updatePositionAndOrientationOf(boxNode, withPosition: boxPosition, relativeTo: cameraNode!)
        updatePositionAndOrientationOf(textNode, withPosition: textPosition, relativeTo: boxNode)
        
        textNode.scale = SCNVector3(0.005,0.005,0.005)
        
        sceneView.scene.rootNode.addChildNode(boxNode)
        sceneView.scene.rootNode.addChildNode(textNode)
        
        currentBoxNode = boxNode
        currentTextNode = textNode
        
        isMovingBox = true
    }
    
    // move node position relative to another node's position.
    func updatePositionAndOrientationOf(_ node: SCNNode, withPosition position: SCNVector3, relativeTo referenceNode: SCNNode) {
        let referenceNodeTransform = matrix_float4x4(referenceNode.transform)
        
        var translationMatrix = matrix_identity_float4x4
        translationMatrix.columns.3.x = position.x
        translationMatrix.columns.3.y = position.y
        translationMatrix.columns.3.z = position.z
        
        let updatedTransform = matrix_multiply(referenceNodeTransform, translationMatrix)
        node.transform = SCNMatrix4(updatedTransform)
    }
    
    // update the location of the box so that it's always in front of the user's camera
    func updateCurrentBoxPosition() {
        let boxPosition = SCNVector3(0,0,-0.6)
        let textPosition = SCNVector3(0,0.1,0)
        let cameraNode = sceneView.pointOfView
        
        updatePositionAndOrientationOf(currentBoxNode, withPosition: boxPosition, relativeTo: cameraNode!)
        updatePositionAndOrientationOf(currentTextNode, withPosition: textPosition, relativeTo: currentBoxNode)
        
        currentTextNode.scale = SCNVector3(0.005,0.005,0.005)
    }
    
    /// Clear tag and pose data
    @objc func clearData() {
        tagData = []
        poseData = []
        moveToButton.setTitleColor(.red, for: .normal)
        explainLabel.text = "Waiting to press start"
        foundTag = false
    }
    
    /// Pop up a view to let the user name their map.
    @objc func popUpSaveView() {
        var mapName = "-"
        let alert = UIAlertController(title: "Info Needed", message: "Enter Recording Name", preferredStyle: .alert)
        alert.addTextField { (textField) in
            textField.placeholder = "Enter Recording Name Here"
        }
        alert.addAction(UIAlertAction(title: "Submit", style: .default, handler: {[weak alert] (_) in
            guard let textField = alert?.textFields?[0], let userText = textField.text else {return}
            mapName = mapName + userText
            self.sendToFirebase(mapName: mapName)
        }))
        
        self.present(alert, animated: true, completion: nil)
    }
    
    /// Upload pose data, last image frame to Firebase under "maps" and "unprocessed_maps" nodes
    func sendToFirebase(mapName: String) {
        let (cameraFrame, timestamp) = getCameraFrame()
        let mapImage = convertToUIImage(cameraFrame: cameraFrame!)
        let mapId = String(timestamp!).replacingOccurrences(of: ".", with: "") + mapName
        let mapJsonFile: [String: Any] = ["map_id": mapId, "camera_intrinsics": getCameraIntrinsics(), "pose_data": poseData, "tag_data": tagData, "location_data": locationData]
        
        let imagePath = "myTestFolder/" + mapId + ".jpg"
        let filePath = "myTestFolder/" + mapId + ".json"
        
        // TODO: handle errors when failing to upload image and json file
        // TODO: let the user pick their image
        // Upload the last image capture to Firebase
        firebaseStorageRef.child(imagePath).putData(UIImageJPEGRepresentation(mapImage, 0)!, metadata: StorageMetadata(dictionary: ["contentType": "image/jpeg"]))

        // Upload raw file json
        if let jsonData = try? JSONSerialization.data(withJSONObject: mapJsonFile, options: []) {
            firebaseStorageRef.child(filePath).putData(jsonData, metadata: StorageMetadata(dictionary: ["contentType": "application/json"])){ (metadata, error) in
                // Write to maps node in database
                self.firebaseRef.child("maps").child(mapId).setValue(["name": mapName, "image": imagePath, "raw_file": filePath])
                
                // Write to unprocessed maps node in database
                self.firebaseRef.child("unprocessed_maps").child(mapId).setValue(filePath)
            }
        }
    }
    
    // record data
    @objc func recordData() {
        let (cameraFrame, timestamp) = getCameraFrame()
        if cameraFrame != nil {
            recordPoseData(cameraFrame: cameraFrame!, timestamp: timestamp!, poseId: poseId)
            recordTags(cameraFrame: cameraFrame!, timestamp: timestamp!, poseId: poseId)
            recordLocationData(cameraFrame: cameraFrame!, timestamp: timestamp!, poseId: poseId)
            poseId += 1
            
            if isMovingBox == true {
                updateCurrentBoxPosition()
            }
        }
    }
    
    /// Append new pose data to list
    @objc func recordPoseData(cameraFrame: ARFrame, timestamp: Double, poseId: Int) {
        poseData.append(getCameraCoordinates(cameraFrame: cameraFrame, timestamp: timestamp, poseId: poseId))
    }
    
    @objc func recordLocationData(cameraFrame: ARFrame, timestamp: Double, poseId: Int) {
        if recordCurrentLocation == true {
            let snapshot = self.sceneView.snapshot()
            let tempLocationData = LocationData(node: currentBoxNode, picture: snapshot, textNode: currentTextNode, poseId: poseId)
            nodeList.append(tempLocationData)
            locationData.append(getLocationCoordinates(cameraFrame: cameraFrame, timestamp: timestamp, poseId: poseId))
            recordCurrentLocation = false
            currentTextNode = SCNNode()
            currentBoxNode = SCNNode()
        }
    }
    
    /// Append new april tag data to list
    @objc func recordTags(cameraFrame: ARFrame, timestamp: Double, poseId: Int) {
        if isProcessingFrame {
            return
        }
        isProcessingFrame = true
        let uiimage = convertToUIImage(cameraFrame: cameraFrame)
        aprilTagQueue.async {
            let arTags = self.getArTags(cameraFrame: cameraFrame, image: uiimage, timeStamp: timestamp, poseId: poseId)
            if !arTags.isEmpty {
                self.tagData.append(arTags)
            }
            self.isProcessingFrame = false
        }
    }
    
    /// Get the current camera frame
    func getCameraFrame() -> (ARFrame?, Double?) {
        let cameraFrame = sceneView.session.currentFrame
        if cameraFrame == nil {
            return (nil, nil)
        }
        return (cameraFrame, cameraFrame!.timestamp)
    }
    
    /// Convert ARFrame to a UIImage
    func convertToUIImage(cameraFrame: ARFrame) -> (UIImage) {
        let pixelBuffer = cameraFrame.capturedImage
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        let context = CIContext(options: nil)
        let cgImage = context.createCGImage(ciImage, from: ciImage.extent)
        let uiImage = UIImage(cgImage: cgImage!)
        return uiImage
    }
    
    /// Get pose data (transformation matrix, time)
    func getCameraCoordinates(cameraFrame: ARFrame, timestamp: Double, poseId: Int) -> [Any] {
        let camera = cameraFrame.camera
        let cameraTransform = camera.transform
        currentFrameTransform = cameraTransform
        let scene = SCNMatrix4(cameraTransform)
        
        let fullMatrix: [Any] = [scene.m11, scene.m12, scene.m13, scene.m14, scene.m21, scene.m22, scene.m23, scene.m24, scene.m31, scene.m32, scene.m33, scene.m34, scene.m41, scene.m42, scene.m43, scene.m44, timestamp, poseId]
        
        return fullMatrix
    }
    
    func getLocationCoordinates(cameraFrame: ARFrame, timestamp: Double, poseId: Int) -> [Any] {
        let locationNode = currentBoxNode
        let nodeTransform = locationNode.simdTransform
        let finalTransform = currentFrameTransform.inverse * nodeTransform
        let scene = SCNMatrix4(finalTransform)
        
        let fullMatrix: [Any] = [scene.m11, scene.m12, scene.m13, scene.m14, scene.m21, scene.m22, scene.m23, scene.m24, scene.m31, scene.m32, scene.m33, scene.m34, scene.m41, scene.m42, scene.m43, scene.m44, timestamp, poseId, locationNode.name!]
        
        return fullMatrix
    }
    
    /// Finds all april tags in the frame
    func getArTags(cameraFrame: ARFrame, image: UIImage, timeStamp: Double, poseId: Int) -> [Any] {
        let intrinsics = cameraFrame.camera.intrinsics.columns
        f.findTags(image, intrinsics.0.x, intrinsics.1.y, intrinsics.2.x, intrinsics.2.y)
        var tagArray: Array<AprilTags> = Array()
        let numTags = f.getNumberOfTags()
        var poseMatrix: [Any] = []
        if numTags > 0 {
            for i in 0...f.getNumberOfTags()-1 {
                tagArray.append(f.getTagAt(i))
            }

            for i in 0...tagArray.count-1 {
                let pose = tagArray[i].poseData
                var simdPose = simd_float4x4(rows: [float4(Float(pose.0), Float(pose.1), Float(pose.2),Float(pose.3)), float4(Float(pose.4), Float(pose.5), Float(pose.6), Float(pose.7)), float4(Float(pose.8), Float(pose.9), Float(pose.10), Float(pose.11)), float4(Float(pose.12), Float(pose.13), Float(pose.14), Float(pose.15))])
                // TODO: it feels like we are doing something wrong here (if we didn't do this transform could we simplify the Invisible Map code?)
                simdPose = simdPose.rotate(radians: Float.pi/2, 0, 0, 1)
                poseMatrix += [tagArray[i].number, simdPose.columns.0.x, simdPose.columns.1.x, simdPose.columns.2.x, simdPose.columns.3.x, simdPose.columns.0.y, simdPose.columns.1.y, simdPose.columns.2.y, simdPose.columns.3.y, simdPose.columns.0.z, simdPose.columns.1.z, simdPose.columns.2.z, simdPose.columns.3.z, simdPose.columns.0.w, simdPose.columns.1.w, simdPose.columns.2.w, simdPose.columns.3.w, timeStamp, poseId]
            }
            DispatchQueue.main.async {
                if self.foundTag == false {
                    self.foundTag = true
                    self.moveToButton.setTitleColor(.blue, for: .normal)
                    self.explainLabel.text = "Tag Found! Now you can save location"
                }
            }
            
        }
        return poseMatrix
    }
        
    /// Get the camera intrinsics
    func getCameraIntrinsics() -> String {
        let camera = sceneView.session.currentFrame?.camera
        let intrinsics = camera?.intrinsics
        let columns = intrinsics?.columns
        let res = camera?.imageResolution
        let width = res?.width
        let height = res?.height
        
        return String(format: "%f,%f,%f,%f,%f,%f,%f", columns!.0.x, columns!.1.y, columns!.2.x, columns!.2.y, columns!.2.z, width!, height!)
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
}
