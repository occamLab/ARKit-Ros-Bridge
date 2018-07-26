# ARKit ROS Bridge

This will stream pose data from an iOS device to a computer via UDP in order to manipulate the data in ROS. Currently, the iOS-ROS streamer sends pose and camera data between the devices.

Built For OccamLab @Olin College 2018

### Running the ARKit ROS Streamer

##### Setting up your computer to run the streamer with ROS:

(1) Enter your catkin workspace directory (For directions on creating a catkin workspace, see http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

(2) Run `roscore` in your terminal

(3) Open a new terminal window and run `roslaunch ios_streamer stream.launch`

(4) To view different visualizations of the data being streamed, you may choose to run either `rosrun rqt_gui rqt_gui` or `rviz` in another terminal window

(5) If you have OccamLab's mobility_games repository cloned to your device, you may choose to run `roslaunch mobility_sensing ar_localization.launch` in order to view visualizations of april tag detections in either rviz or the rqt gui. You will need to first change the namespace for the apriltags_ros package from `camera` to `camera_lowres`. You may also want to add tag descriptions for the april tags.

##### Running the streamer on an iOS device:

(1) You will need to create an Apple id and register yourself as an iOS developer. After cloning this repository to a Mac, you can create an app by opening the `iOS_ros_stream.xcodeproj` folder in Xcode. For reference, the streamer was originally developed in Xcode 9.4.1. 

(2) Connect your Mac to an iOS device and build the app to that device.

(3) On the iOS device, connect to the WiFi network to which the computer running ROS will be connected.

(4) Open the streamer app on the iOS device, input the ip address of the computer to which you intend to send data, and press the "Transmit Data" button. If your computer is running the streamer, it should now be receiving data from your iOS device.

If you find the streamer has significant lag, please try relaunching the iOS app.


#### Attributions:
The following provides the basis for how data is sent from an iOS device via UDP connection to a computer:
https://github.com/gunterhager/UDPBroadcastConnection

To learn more about OccamLab, please visit our website: http://occam.olin.edu/.
