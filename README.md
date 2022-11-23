# ARKit ROS Bridge

This will stream data from an iOS device to a computer via UDP in order to manipulate the data in ROS.

Built For OccamLab @Olin College 2018


$$e^x$$
$$e^{2x}$$


### Running the ARKit ROS Streamer

##### Setting up your computer to run the streamer with ROS:

(1) Enter your catkin workspace directory (For directions on creating a catkin workspace, see "First Time Setup" below.) and clone this repository into `catkin_ws/src` as arkit_streamer.

(2) In your `catkin_ws/src` directory, you will need to run `catkin_create_pkg arkit_streamer`. Then, in your `CMakeLists.txt` file in the `arkit_streamer` directory, you will need to modify your requirements to read:
```
  find_package(catkin REQUIRED COMPONENTS
  apriltags_ros
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  message_generation
)
```
You will also need to uncomment the dependencies and add `geometry_msgs` and `sensor_msgs`. Furthermore, you will need to uncomment the following lines:
```
add_service_files(
 FILES
 Service1.srv
 Service2.srv
)

```
And replace the placeholder `Service*.srv` with `phone.srv`.
Finally, in your `package.xml` file, you will need to copy the following below the line reading `<buildtool_depend>catkin</buildtool_depend>`:
  ```
    <build_depend>apriltags_ros</build_depend>
    <build_depend>geometry_msgs</build_depend>
    <build_depend>rospy</build_depend>
    <build_depend>sensor_msgs</build_depend>
    <build_depend>std_msgs</build_depend>
    <build_depend>message_generation</build_depend>
    <build_export_depend>apriltags_ros</build_export_depend>
    <build_export_depend>geometry_msgs</build_export_depend>
    <build_export_depend>rospy</build_export_depend>
    <build_export_depend>sensor_msgs</build_export_depend>
    <build_export_depend>std_msgs</build_export_depend>
    <exec_depend>apriltags_ros</exec_depend>
    <exec_depend>geometry_msgs</exec_depend>
    <exec_depend>rospy</exec_depend>
    <exec_depend>sensor_msgs</exec_depend>
    <exec_depend>std_msgs</exec_depend>
    <exec_depend>message_runtime</exec_depend>
  ```

(3) Run `roscore` in your terminal

(4) Open a new terminal window and run `roslaunch arkit_streamer stream.launch`

(5) To view different visualizations of the data being streamed, you may choose to run either `rosrun rqt_gui rqt_gui` or `rviz` in another terminal window. In rviz, you will see two frames appear when an april tag is detected if you send the phone's pose, camera feed, and the april tags it detects to ROS. The first will be called tag_(insert number), which represents april tags detected by the apriltags_ros package, and the second will be called ios_tag_(insert number), which represents april tags detected by the visual servoing platform on the iPhone itself.

##### Running the streamer on an iOS device:

(1) After cloning this repository as arkit_streamer, go to http://visp-doc.inria.fr/download/snapshot/ios/ and download and unzip the latest version of the visp3 framework.

(2) In Finder, drag the `opencv2.framework` and `visp3.framework` frameworks into your local arkit_streamer folder.

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
