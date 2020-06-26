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

#### With PySceneNetRGBD

* start the dataset publisher ``` ros2 run rosquadricslam dataset_publisher ```
* start the system ``` ros2 run rosquadricslam system ```

#### With webcam

<!-- * start the webcam publisher ``` ros2 run image_tools cam2image --ros-args -p width:=640 -p height:=480 ``` -->
* start the webcam publisher ``` ros2 run rosquadricslam webcam_publisher --ros-args -p width:=640 -p height:=480 ```
* start the object detector ``` ros2 run py_detector detect ```
* start the slam system ``` ros2 run openvslam run_slam -v ../../example/webcam/orb_vocab.dbow2 -c ../../example/webcam/mono.yaml ```
* start the quadricslam system ``` ros2 run rosquadricslam system  ```
