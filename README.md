# FYP18/19- Lidar Inertial Odometry Localization of UAV in A Tunnel-like Environment
## Overview
This is a Final Year Project under School of Mechanical and Aerospace Engineering, NTU. 

The objective is to implement the suitable SLAM algorithm for localization of an UAV.
The UAV platform carries a 2D LIDAR sensor and onboard Pixhawk fcu as IMU data source. Additional TFmini range finder is installed to detect the altitude.

This repo consists of cpp in src/beginner_tutorials folder: 
- Lidar Fusion
- TF Braodcaster
- Mocap bridge
- Px4 pos control
- state RCout

## Prerequisites
In this repo, we only focus on the Hector SLAM algorithm with RPLidar sensor. Please self-source for other potential SLAM packages if interested.
### Packages
4 packages to be installed (kinetic version)
- [rplidar_ros](http://wiki.ros.org/rplidar)
- [hector_slam](http://wiki.ros.org/hector_slam)
- [mavros](https://dev.px4.io/en/ros/mavros_installation.html)
- [mavlink](https://dev.px4.io/en/ros/mavros_installation.html)

## Implementation
After necessary packages are installed in `catkin_ws`, use `catkin build` to build the binary executable.
### rplidar_ros
Visualise the laser scan via Rviz: 
```
roslaunch rplidar_ros view_rplidar.launch
```
### hector_slam
To perform full stack of hector SLAM, please follow detailed instructions in the link above.
You will need to add some launch files from `/src/config_hector` to `src/hector_slam/hector_slam/launch`, then rebuild.

To launch without IMU fusion:
```
roslaunch hector_slam tutorial_withoutimu.launch
```

To launch with IMU fusion:
```
roslaunch hector_slam tutorial_imu.launch
```

### mavros
Receive Pixhawk data via ROS:
```
roslaunch mavros px4.launch
```
If you are not able to run this node, please check the connection of your Pixhawk with
```
ls /dev/tty*
```
then you should see '/dev/ttyACM0'. Or else please check extra.txt in SD card on Pixhawk.

## Launch Files
### 1. lidar_fusion.launch
This node will subscribe to `/slam_out_pose` , `/mavros/distance_sensor/tfmini_pub` and `/mavros/local_pose/pose`, then publish 3D pose to topic
`/mavros/vision_pose/pose`.

To launch:
```
roslaunch beginner_tutorials lidar_fusion.launch
```
#### Parameters
- `loop_rate` - The update frequency of pose information, in Hz.
  - Default: 10
- `base_frame` - The frame id of reference
  - Default: "map"
### 2. tf_broadcaster.launch
This node will convert the `geometry_msgs/PoseStamped` to `tf`
To launch:
```
roslaunch beginner_tutorials tf_broadcaster.launch
```
### 3. mocap_bridge.launch
This node will receive the data from Mocap System.
To launch:
```
roslaunch mocap_bridge mocap_bridge.launch
```

> To change the parameters, direct edit the .launch files in `./launch` folder respectively

## Detail Information
For more detailed documentation, please refer to [here](documentary/C018_SLAM.pdf)
      
