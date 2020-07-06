# Ros2 packages
We provide an example of running QuadricSLAM live using odometry measurements from OpenVSLAM (C++) and object detections from Yolov3 (Python/PyTorch). We have chosen to use ROS2 in order to make the system easily modified to use odometry / detections from any source. We provide 4 ROS packages outlined below:

* detection_msgs: contains object detection messages published at '/detections' from the py_detector 
* py_detector: wraps eriklindernoren's pytorch implementation of yolov3, but can be easily modified to use any existing pytorch detector. Subscribes to '/image' and publishes detections to '/detections'. 
* rosopenvslam: wraps openvslam to subscribe to '/image' and publish to '/poses'. 
* rosquadricslam: provides a webcam publisher (similar to image_tools cam2image but with correct timestamps), a dataset publisher, and the core slam system. 

## Installation 
### Dependencies

* [OpenVSLAM](https://github.com/xdspacelab/openvslam)
* [PyTorch-YOLOv3](https://github.com/eriklindernoren/PyTorch-YOLOv3) 
* [ROS2](https://index.ros.org/doc/ros2/Installation/) w/ packages: [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2), [image_transport](https://github.com/ros-perception/image_common/tree/ros2)
* Python3 w/ modules: pytorch, pillow, cv2, numpy, matplotlib

### Instructions

* Clone and build [QuadricSLAM](/README.md) with QSLAM_BUILD_PYTHON_WRAP set ON
    * Test QuadricSLAM is working with `python3 examples/python/simple_example.py`
* Follow the [OpenVSLAM installation guide](https://openvslam.readthedocs.io/en/master/installation.html)
    * Ensure you build with **OpenCV 3.x**
    * When you have [calibrated your camera](https://github.com/ros-perception/image_pipeline/tree/ros2) store the calibration as demonstrated in the openvslam examples (`openvslam/example/tum_rgbd/TUM_RGBD_mono_1.yaml`). 
    * Follow the [SLAM with UVC camera](https://openvslam.readthedocs.io/en/master/example.html) example
    * Test OpenVSLAM is working with a webcam using `./run_camera_slam -v /path/to/orb_vocab.dbow2 -c /path/to/webcam_config.yaml -n 0`
* Clone PyTorch-YOLOv3 and add /path/to/PyTorch-YOLOv3/ to PYTHONPATH
    * Install the requirements listed in requirements.txt
    * Follow the instructions to download the pretrained Yolov3 weights
    * Test the detector is working with `python3 detect.py --image_folder data/samples`
* Install ROS2 and additional packages:
    * Follow the [ROS installation guide](https://index.ros.org/doc/ros2/Installation/)
    * Create a new ros2 workspace or use /quadricslam/ros/src
    * Clone the ROS2 branch of vision_opencv (for cv_bridge) and image_common (for image_transport)
    * Build the workspace `colcon build --symlink-install`
    * Source the ROS2 overlay `. install/setup.bash` 
* Navigate to /quadricslam/ros/src and build the ros packages with `colcon build --symlink-install`
* Source the overlay with `. install/setup.bash`

## Usage with USB Camera

* Start the webcam publisher: `ros2 run rosquadricslam webcam_publisher --ros-args -p width:=640 -p height:=480`
* Start the object detector:  `ros2 run py_detector detect --ros-args -p weights:=/path/to/yolov3.weights -p config:=/path/to/yolov3.cfg -p classes:=/path/to/coco.names`
* Start OpenVSLAM tracking: `ros2 run rosopenvslam run_slam -v /path/to/orb_vocab.dbow2 -c /path/to/webcam_config.yaml`
* Start QuadricSLAM mapping: `ros2 run rosquadricslam system` 

## Usage with PyScenenetRGBD

* Start the dataset publisher: `ros2 run rosquadricslam dataset_publisher`
* Start QuadricSLAM mapping: `ros2 run rosquadricslam system` 

## Common Issues

If you find that ROSQuadricSLAM is not receiving any messages from /image, /detections and /poses, it might be because the TimeSynchronizer is giving up because the py_detector is taking too long to republish /detections. Ensure you have built pytorch with GPU support. 
