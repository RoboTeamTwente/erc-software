# ArUco Marker Broadcaster

An application based on OpenCV 4.6.0 (to comply with current ROS2 version of ERC-Software) and ROS2 Kilted Kaiju.

Displays a live video stream of detected ArUco IDs.

The application is a ROS2 node that publishes the pose estimation of the detected ArUco codes after TF2 transformations.

To be used in the Navigation or Maintenence Subtasks of ERC.

## Running the node

Every time you update the code within 'src/aruco_tf2_node.cpp' you must rebuild and source your terminal.

### Step 1 - Build

```bash
colcon build --packages-select aruco_track
```

### Step 2 - Source the terminal

```bash
. ./install/setup.bash
```

```bash
. /opt/ros/kilted/setup.bash
```

### Step 3 - Run the ROS2 Node

```bash
ros2 run aruco_track aruco_tf2_node
```

### Step 4 - Select Video Source

Self explanatory based on the terminal print message:

```bash
=== ArUco Tracker ===
Select video source:
  [ENTER]  Use default USB camera (index 0)
  [0/1/2]  Use a different camera index
  [URL]    Use an IP camera / stream URL

Examples:
  http://192.168.0.152:8080/video   (IP Webcam)
  0
```

Default value is index 0. This will use the camera connected on '/video0'. Other values will use different outputs, '/video1' or '/video2' and so on if you have multiple connected cameras.

IP Webcam means using a mobile application called "IP Webcam" that starts a local server to transmit your mobile phone's camera feed to your computer. The IP will frequently change when you start the server.

#### *'I don't see the video output window'*

Unless you point the camera at a 5x5 (150mm) ArUco marker, the window will not appear. You can tell you are not detecting any markers by looking at the warning: `[WARN] [1767900227.913066660] [aruco_tf2_node]: No markers found`.

### Step 5 - View Detected ArUco Poses

You can run the following command to see the detected ArUco IDs and pose estimations right within your terminal, however **the aruco_tf2_node must be running in parallel in a separate terminal**:

```bash
ros2 topic echo /tf
```

Alternatively, you can view the pose estimation with **rviz**.

You can expect a simillar output:

```bash
---
transforms:
- header:
    stamp:
      sec: 1767899812
      nanosec: 662838346
    frame_id: camera_frame
  child_frame_id: aruco_11
  transform:
    translation:
      x: -0.03687934682250437
      y: -0.012666235570217378
      z: 0.13659864132869448
    rotation:
      x: -0.6982514505098149
      y: 0.7123967508805725
      z: -0.0574988868747503
      w: 0.04036903768865494
---
```

## TODO: Camera Calibration

The CharUco camera calibration cpp file somehow vanished during development. It must be reimplemented to obtain a better `charuco_camera_params.yml` file with more optimized parameters.
