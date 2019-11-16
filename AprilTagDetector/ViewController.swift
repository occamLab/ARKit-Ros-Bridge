//
//  ViewController.swift
//  AprilTagDetector
//
//  Created by djconnolly27 on 7/11/18.
//  Copyright © 2018 OccamLab. All rights reserved.
//

import UIKit
import ARKit
import FirebaseDatabase
import FirebaseStorage

class ViewController: UIViewController {

    //MARK: Properties

    @IBOutlet var sceneView: ARSCNView!
    
    @IBOutlet weak var ipAddressText: UITextField!
    
    @IBOutlet var poseSwitch: UISwitch!
    @IBOutlet var imageSwitch: UISwitch!
    @IBOutlet var aprilTagSwitch: UISwitch!
    
    var broadcastTags: UDPBroadcastConnection!
    var broadcastPoseConnection: UDPBroadcastConnection!
    var broadcastImagesConnection: UDPBroadcastConnection!
    
    var poseTimer = Timer()
    var imageTimer = Timer()
    var tagFinderTimer = Timer()
    
    var isProcessingFrame = false
    var imageIndex = 0                 // this is the sequence number in the image stream
    let f = imageToData()
    let aprilTagQueue = DispatchQueue(label: "edu.occamlab.apriltagfinder", qos: DispatchQoS.userInitiated)
    
    var firebaseRef: DatabaseReference!
    var firebaseStorage: Storage!
    var firebaseStorageRef: StorageReference!
    
    /// Begin an ARSession when the app first loads
    override func viewDidLoad() {
        super.viewDidLoad()
        startSession()
        firebaseRef = Database.database().reference()
        firebaseStorage = Storage.storage()
        firebaseStorageRef = firebaseStorage.reference()
        let tap: UITapGestureRecognizer = UITapGestureRecognizer(target: self, action: #selector(dismissKeyboard))
        view.addGestureRecognizer(tap)
    }
    
    func writeMapToFirebase(mapName: String, mapImage: UIImage, mapJsonFile: [String: Any]) {
        let imagePath = "images/" + mapName + ".jpg"
        let filePath = "files/" + mapName + ".json"
        
        // TODO: handle errors when failing to upload image and json file
        // Upload image
        firebaseStorageRef.child(imagePath).putData(UIImageJPEGRepresentation(mapImage, 0)!, metadata: StorageMetadata(dictionary: ["contentType": "image/jpeg"]))

        // Upload result json
        if let jsonData = try? JSONSerialization.data(withJSONObject: mapJsonFile, options: []) {
            firebaseStorageRef.child(filePath).putData(jsonData, metadata: StorageMetadata(dictionary: ["contentType": "application/json"]))
        }

        // Write to maps node in database
        firebaseRef.child("maps").child(mapName).setValue(["image": imagePath, "map_file": filePath])
    }
    
    /// Resigns keyboard when screen is tapped
    @objc func dismissKeyboard() {
        view.endEditing(true)
        setupUdpConnections()
    }
    
    /// Initialize the ARSession
    func startSession() {
        let configuration = ARWorldTrackingConfiguration()
        sceneView.session.run(configuration)
    }
    
    /// Initializes UDP connections for sending data to ROS
    func setupUdpConnections() {
        let INADDR_BROADCAST = in_addr(s_addr: inet_addr(ipAddressText.text))
        
        broadcastPoseConnection = UDPBroadcastConnection(port: Config.Ports.broadcast, ip: INADDR_BROADCAST) {(port: Int, response: [UInt8]) -> Void in
            print("Received from \(INADDR_BROADCAST):\(port):\n\n\(response)")
        }
        
        broadcastImagesConnection = UDPBroadcastConnection(port: Config.Ports.broadcastImages, ip: INADDR_BROADCAST) {(port: Int, response: [UInt8]) -> Void in
            print("Received from \(INADDR_BROADCAST):\(port):\n\n\(response)")
        }
        
        broadcastTags = UDPBroadcastConnection(port: Config.Ports.broadcastAprilTags, ip: INADDR_BROADCAST) {(port: Int, response: [UInt8]) -> Void in
            print("Received from \(INADDR_BROADCAST):\(port):\n\n\(response)")
        }
    }
    
    /// Sends selected types of data to ROS when button is pressed
    @IBAction func startButtonTapped(_ sender: UIButton) {
        if sender.currentTitle == "Start" {
            sender.setTitle("Stop", for: .normal)
            
            if poseSwitch.isOn {
                poseTimer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(self.transmitPoseData), userInfo: nil, repeats: true)
            }
            if imageSwitch.isOn {
                imageTimer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(self.transmitImages), userInfo: nil, repeats: true)
            }
            if aprilTagSwitch.isOn {
                tagFinderTimer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(self.transmitTags), userInfo: nil, repeats: true)
            }

            poseSwitch.addTarget(self, action: #selector(scheduledTimerToTransmitData), for: .valueChanged)
            imageSwitch.addTarget(self, action: #selector(scheduledTimerToTransmitData), for: .valueChanged)
            aprilTagSwitch.addTarget(self, action: #selector(scheduledTimerToTransmitData), for: .valueChanged)
        }
        else {
            sender.setTitle("Start", for: .normal)
            poseTimer.invalidate()
            poseTimer = Timer()
            poseSwitch.removeTarget(self, action: #selector(scheduledTimerToTransmitData), for: .valueChanged)

            imageTimer.invalidate()
            imageTimer = Timer()
            imageSwitch.removeTarget(self, action: #selector(scheduledTimerToTransmitData), for: .valueChanged)
            
            tagFinderTimer.invalidate()
            tagFinderTimer = Timer()
            aprilTagSwitch.removeTarget(self, action: #selector(scheduledTimerToTransmitData), for: .valueChanged)
        }

    }
    
    /// Initializes timers to send data at regular intervals
    @objc func scheduledTimerToTransmitData() {
        print("Checking to see what to transmit")
        poseTimer.invalidate()
        poseTimer = Timer()

        if poseSwitch.isOn {
            poseTimer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(self.transmitPoseData), userInfo: nil, repeats: true)
        }
        imageTimer.invalidate()
        imageTimer = Timer()

        if imageSwitch.isOn {
            imageTimer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(self.transmitImages), userInfo: nil, repeats: true)
        }
        tagFinderTimer.invalidate()
        tagFinderTimer = Timer()

        if aprilTagSwitch.isOn {
            tagFinderTimer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(self.transmitTags), userInfo: nil, repeats: true)
        }
    }
    
    /// Sends the pose data to ROS
    @objc func transmitPoseData() {
        broadcastPoseConnection.sendBroadcast(getCameraCoordinates())
    }
    
    /// Sends the april tag data to ROS
    @objc func transmitTags() {
        if isProcessingFrame {
            return
        }
        isProcessingFrame = true
        let (image, timeStamp) = getVideoFrames()
        let rotatedImage = imageRotatedByDegrees(oldImage: image, deg: 90)
        aprilTagQueue.async {
            self.broadcastTags.sendBroadcast(self.getArTags(rotatedImage: rotatedImage, timeStamp: timeStamp))
            self.isProcessingFrame = false
        }
    }
    
    /// Sends the camera frames to ROS
    @objc func transmitImages() {
        let intrinsics = getCameraIntrinsics()
        let MTU = 1350
        let (image, stampedTime) = getVideoFrames()
        let imageData = UIImageJPEGRepresentation(image, 0)
        let frameTime = String(stampedTime).data(using: .utf8)!
        let timeAndIntrinsics = frameTime + intrinsics
        var bytesSent = 0           // Keeps track of how much of the image has been sent
        var packetIndex = 0         // Packet number - so ROS can recompile the image in order
        
        while bytesSent < imageData!.count {
            // Construct the range for the packet
            let range = (bytesSent..<min(bytesSent + MTU, imageData!.count))
            var udpPacketPayload = imageData!.subdata(in: range)
            udpPacketPayload.insert(UInt8(imageIndex % (Int(UInt8.max) + 1)), at: 0)
            udpPacketPayload.insert(UInt8(packetIndex), at: 1)
            
            if bytesSent == 0 {
                let numPackets = (Float(imageData!.count) / Float(MTU)).rounded(.up)
                udpPacketPayload.insert(UInt8(numPackets), at: 2)
                udpPacketPayload.insert(UInt8(frameTime.count), at: 3)
                udpPacketPayload.insert(UInt8(intrinsics.count), at: 4)
                udpPacketPayload.insert(contentsOf: timeAndIntrinsics, at: 5)
            }
            broadcastImagesConnection.sendBroadcast(udpPacketPayload)
            bytesSent += range.count
            packetIndex += 1
        }
        imageIndex += 1
    }
    
    /// Get video frames.
    func getVideoFrames() -> (UIImage, Double) {
        let cameraFrame = sceneView.session.currentFrame
        let stampedTime = cameraFrame?.timestamp
        
        // Convert ARFrame to a UIImage
        let pixelBuffer = cameraFrame?.capturedImage
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer!)
        let context = CIContext(options: nil)
        let cgImage = context.createCGImage(ciImage, from: ciImage.extent)
        let uiImage = UIImage(cgImage: cgImage!)
        return (uiImage, stampedTime!)
    }
    
    /// Get pose data (transformation matrix, time) and send to ROS.
    func getCameraCoordinates() -> String {
        let camera = sceneView.session.currentFrame?.camera
        let cameraTransform = camera?.transform
        let relativeTime = sceneView.session.currentFrame?.timestamp
        let scene = SCNMatrix4(cameraTransform!)
        
        let fullMatrix = String(format: "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", scene.m11, scene.m12, scene.m13, scene.m14, scene.m21, scene.m22, scene.m23, scene.m24, scene.m31, scene.m32, scene.m33, scene.m34, scene.m41, scene.m42, scene.m43, scene.m44, relativeTime!)
        
        return fullMatrix
    }
    
    /// Finds all april tags in the frame to send to ROS
    func getArTags(rotatedImage: UIImage, timeStamp: Double) -> String {
        let intrinsics = sceneView.session.currentFrame?.camera.intrinsics.columns
        f.findTags(rotatedImage, intrinsics!.1.y, intrinsics!.0.x, intrinsics!.2.y, intrinsics!.2.x)
        var tagArray: Array<AprilTags> = Array()
        let numTags = f.getNumberOfTags()
        var poseMatrix = "START"
        if numTags > 0 {
            for i in 0...f.getNumberOfTags()-1 {
                tagArray.append(f.getTagAt(i))
            }

            for i in 0...tagArray.count-1 {
                let pose = tagArray[i].poseData
                poseMatrix = poseMatrix + "," + String(format: "TAG,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", tagArray[i].number, pose.0, pose.1, pose.2, pose.3, pose.4, pose.5, pose.6, pose.7, pose.8, pose.9, pose.10, pose.11, pose.12, pose.13, pose.14, pose.15, timeStamp)
            }
        }
        return poseMatrix
    }
        
    /// Get the camera intrinsics to send to ROS
    func getCameraIntrinsics() -> Data {
        let camera = sceneView.session.currentFrame?.camera
        let intrinsics = camera?.intrinsics
        let columns = intrinsics?.columns
        let res = camera?.imageResolution
        let width = res?.width
        let height = res?.height
        
        return String(format: "%f,%f,%f,%f,%f,%f,%f", columns!.0.x, columns!.1.y, columns!.2.x, columns!.2.y, columns!.2.z, width!, height!).data(using: .utf8)!
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
