# dataset_publisher

Loads SceneNetRGBD and publishes to /image /detections /poses

## Dependencies 
* numpy 
* cv2
* matplotlib
* ROS2
* cv_bridge
* detection_msgs
* example_frontend (for containers and scenenet interface)

## Parameters

* delay (float): time in seconds between publishing dataset frames 

## Usage

First ensure you have sourced ROS2 overlays with the cv_bridge package. 
Then you can use `ros2 run rosquadricslam dataset_publisher` to start the dataset publisher.
Alternatively `ros2 run rosquadricslam dataset_publisher --ros-args -p delay:=0.2` to change the rate. 


# webcam_publisher 

## Dependencies 

* numpy
* cv2
* ROS2
* cv_bridge

## Parameters

* width (float): desired webcam image width
* height (float): desired webcam image height
* fps (float): the target webcam fps. However, actual fps could be lower depending on max fps of the webcam. 

## Usage

First ensure you have sourced ROS2 overlays with the cv_bridge package. 
Then you can use `ros2 run rosquadricslam webcam_publisher --ros-args -p width:=640 -p height:=480` to start the webcam publisher.