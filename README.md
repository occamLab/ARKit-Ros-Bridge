# ARKit ROS Bridge

This will stream data from an iOS device to a computer via UDP in order to manipulate the data in ROS.

Built For OccamLab @Olin College 2018

### Running the ARKit ROS Streamer

##### Setting up your computer to run the streamer with ROS:

(1) Enter your catkin workspace directory (For directions on creating a catkin workspace, see "First Time Setup" below.) and clone this repository into `catkin_ws/src` as arkit_streamer.

(2) Run `roscore` in your terminal

(3) Open a new terminal window and run `roslaunch arkit_streamer stream.launch`

(4) To view different visualizations of the data being streamed, you may choose to run either `rosrun rqt_gui rqt_gui` or `rviz` in another terminal window. In rviz, you will see two frames appear when an april tag is detected if you send the phone's pose, camera feed, and the april tags it detects to ROS. The first will be called tag_(insert number), which represents april tags detected by the apriltags_ros package, and the second will be called ios_tag_(insert number), which represents april tags detected by the visual servoing platform on the iPhone itself.

##### Running the streamer on an iOS device:

(1) After cloning this repository, go to http://visp-doc.inria.fr/download/snapshot/ios/ and download and unzip the latest version of the visp3 framework.

(2) In Finder, drag the `opencv2.framework` and `visp3.framework` frameworks into your local AprilTagDetector folder.

(3) Open the `AprilTagDetector.xcodeproj` folder in Xcode. For reference, the streamer was originally developed in Xcode 9.4.1. 

(4) If you have an iOS developer account, go to Project Settings, and under General, select your team and signing certificate. If you do not have an iOS developer account, you can create one by following the instructions found https://developer.apple.com/programs/enroll/.

(5) Connect an iOS device to your Mac computer and build the app on the device.

(6) On the iOS device, connect to the WiFi network to which the computer running ROS will be connected.

(7) Open the streamer app on the iOS device, input the ip address of the computer to which you intend to send data, and press the "Start" button. You can then select which types of data (pose, images, or april tags) you would like to transmit to your computer. If your computer is running the streamer, it should now be receiving data from your iOS device.

###### First time setup

(1) Make sure you have a catkin workspace! (If you don't, here are [instructions for setting up a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)).

(2) Clone this directory into `catkin_ws/src`.

(3) Enter your `catkin_ws` direction and run the following commands:

```bash
$ catkin_make install
$ source devel/setup.bash
```

(4) Follow instructions above from step (2)!


#### Attributions:
The following provides the basis for how data is sent from an iOS device via UDP connection to a computer:
https://github.com/gunterhager/UDPBroadcastConnection

To learn more about OccamLab, please visit our website: http://occam.olin.edu/.
