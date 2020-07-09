# py_detector 

Subscribes to /image and publishes to /detections

## Dependencies 
* numpy 
* PIL
* cv2
* torch
* matplotlib
* ROS2
* cv_bridge
* detection_msgs

## Usage

First ensure you have sourced ROS2 overlays with cv_bridge and detection_msgs packages. 
Then you can use `ros2 run py_detector detect` to start the detector.