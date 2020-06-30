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

class ViewController: UIViewController, writeValueBackDelegate {
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
    var poseId: Int = 0
    
    var firebaseRef: DatabaseReference!
    var firebaseStorage: Storage!
    var firebaseStorageRef: StorageReference!
    
    //boolean to check whether the camera found tag or not
    var foundTag: Bool = false
    
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
        if sender.currentTitle == "Start" {
            sender.setTitle("Stop", for: .normal)
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
                sender.setTitle("Start", for: .normal)
            })
        }

    }
    /// Move to the save location screen
    @IBAction func moveButtonTapped(_ sender: Any) {
        if foundTag == true {
            self.performSegue(withIdentifier: "Show", sender: self)
        } else {
            explainLabel.text = "You first have to find a tag"
        }
    }
    
    // when moving to the other view, prepare delegate to get the data back
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        let saveLocationController = segue.destination as! SaveLocationController
        saveLocationController.delegate = self
    }
    
    // function called when dismissing the save location view
    func writeValueBack(value: [String:Any]) {
        let tempString = value["LocationName"] as! String
        explainLabel.text = tempString
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
        // TODO: Create a real pop up view!
        let mapName = "-Test"
        sendToFirebase(mapName: mapName)
    }
    
    /// Upload pose data, last image frame to Firebase under "maps" and "unprocessed_maps" nodes
    func sendToFirebase(mapName: String) {
        let (cameraFrame, timestamp) = getCameraFrame()
        let mapImage = convertToUIImage(cameraFrame: cameraFrame!)
        let mapId = String(timestamp!).replacingOccurrences(of: ".", with: "") + mapName
        let mapJsonFile: [String: Any] = ["map_id": mapId, "camera_intrinsics": getCameraIntrinsics(), "pose_data": poseData, "tag_data": tagData]
        
        let imagePath = "myTestFolder/" + mapId + ".jpg"
        let filePath = "myTestFolder/" + mapId + ".json"
        
        // TODO: handle errors when failing to upload image and json file
        // TODO: let the user pick their image
        // Upload the last image capture to Firebase
        firebaseStorageRef.child(imagePath).putData(UIImageJPEGRepresentation(mapImage, 0)!, metadata: StorageMetadata(dictionary: ["contentType": "image/jpeg"]))

        // Upload raw file json
        if let jsonData = try? JSONSerialization.data(withJSONObject: mapJsonFile, options: []) {
            firebaseStorageRef.child(filePath).putData(jsonData, metadata: StorageMetadata(dictionary: ["contentType": "application/json"]))
        }

        // Write to maps node in database
        firebaseRef.child("maps").child(mapId).setValue(["name": mapName, "image": imagePath, "raw_file": filePath])
        
        // Write to unprocessed maps node in database
        firebaseRef.child("unprocessed_maps").child(mapId).setValue(filePath)
    }
    
    // record data
    @objc func recordData() {
        let (cameraFrame, timestamp) = getCameraFrame()
        if cameraFrame != nil {
            recordPoseData(cameraFrame: cameraFrame!, timestamp: timestamp!, poseId: poseId)
            recordTags(cameraFrame: cameraFrame!, timestamp: timestamp!, poseId: poseId)
            poseId += 1
        }
    }
    
    /// Append new pose data to list
    @objc func recordPoseData(cameraFrame: ARFrame, timestamp: Double, poseId: Int) {
        poseData.append(getCameraCoordinates(cameraFrame: cameraFrame, timestamp: timestamp, poseId: poseId))
    }
    
    /// Append new april tag data to list
    @objc func recordTags(cameraFrame: ARFrame, timestamp: Double, poseId: Int) {
        if isProcessingFrame {
            return
        }
        isProcessingFrame = true
        let uiimage = convertToUIImage(cameraFrame: cameraFrame)
        let rotatedImage = imageRotatedByDegrees(oldImage: uiimage, deg: 90)
        aprilTagQueue.async {
            let arTags = self.getArTags(cameraFrame: cameraFrame, rotatedImage: rotatedImage, timeStamp: timestamp, poseId: poseId)
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
        let scene = SCNMatrix4(cameraTransform)
        
        let fullMatrix: [Any] = [scene.m11, scene.m12, scene.m13, scene.m14, scene.m21, scene.m22, scene.m23, scene.m24, scene.m31, scene.m32, scene.m33, scene.m34, scene.m41, scene.m42, scene.m43, scene.m44, timestamp, poseId]
        
        return fullMatrix
    }
    
    /// Finds all april tags in the frame
    func getArTags(cameraFrame: ARFrame, rotatedImage: UIImage, timeStamp: Double, poseId: Int) -> [Any] {
        let intrinsics = cameraFrame.camera.intrinsics.columns
        f.findTags(rotatedImage, intrinsics.1.y, intrinsics.0.x, intrinsics.2.y, intrinsics.2.x)
        var tagArray: Array<AprilTags> = Array()
        let numTags = f.getNumberOfTags()
        var poseMatrix: [Any] = []
        if numTags > 0 {
            for i in 0...f.getNumberOfTags()-1 {
                tagArray.append(f.getTagAt(i))
            }

            for i in 0...tagArray.count-1 {
                let pose = tagArray[i].poseData
                poseMatrix += [tagArray[i].number, pose.0, pose.1, pose.2, pose.3, pose.4, pose.5, pose.6, pose.7, pose.8, pose.9, pose.10, pose.11, pose.12, pose.13, pose.14, pose.15, timeStamp, poseId]
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
    
    /// Rotates an image clockwise
    func imageRotatedByDegrees(oldImage: UIImage, deg degrees: CGFloat) -> UIImage {
        //Calculate the size of the rotated view's containing box for our drawing space
        let rotatedViewBox: UIView = UIView(frame: CGRect(x: 0, y: 0, width: oldImage.size.width, height: oldImage.size.height))
        let t: CGAffineTransform = CGAffineTransform(rotationAngle: degrees * CGFloat.pi / 180)
        rotatedViewBox.transform = t
        let rotatedSize: CGSize = rotatedViewBox.frame.size
        //Create the bitmap context
        UIGraphicsBeginImageContext(rotatedSize)
        let bitmap: CGContext = UIGraphicsGetCurrentContext()!
        //Move the origin to the middle of the image so we will rotate and scale around the center.
        bitmap.translateBy(x: rotatedSize.width / 2, y: rotatedSize.height / 2)
        //Rotate the image context
        bitmap.rotate(by: (degrees * CGFloat.pi / 180))
        //Now, draw the rotated/scaled image into the context
        bitmap.scaleBy(x: 1.0, y: -1.0)
        bitmap.draw(oldImage.cgImage!, in: CGRect(x: -oldImage.size.width / 2, y: -oldImage.size.height / 2, width: oldImage.size.width, height: oldImage.size.height))
        let newImage: UIImage = UIGraphicsGetImageFromCurrentImageContext()!
        UIGraphicsEndImageContext()
        return newImage
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
}
