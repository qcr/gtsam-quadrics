# Ros2 packages
We provide an example of running QuadricSLAM live using odometry measurements from OpenVSLAM (C++) and object detections from Yolov3 (Python/PyTorch). We have chosen to use ROS2 in order to make the system easily modified to use odometry / detections from any source. 

## Installation
### Requirements
* openvslam: install develop branch for ROS2 support
* ROS2

### Build Instructions
```
cd /path/to/quadricslam/ros/
colcon build
```

### Usage

* start the camera publisher (ros2 run image_tools cam2image --ros-args -p width:=640 -p height:=480)
* start the detector (ros2 run py_detector detect)

```
source /path/to/quadricslam/ros/install/setup.bash
ros2 run py_detector listener
```

