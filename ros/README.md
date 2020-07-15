# Ros2 packages
We provide an example of running QuadricSLAM live using odometry measurements from OpenVSLAM (C++) and object detections from Yolov3 (Python/PyTorch). We have chosen to use ROS2 in order to make the system easily modified to use odometry / detections from any source. We provide 4 ROS packages outlined below:

* detection_msgs: contains object detection messages published at '/detections' from the py_detector 
* webcam_publisher: similar to image_tools cam2image but with correct timestamps, publishes to '/images'
* yolov3_ros: wraps [eriklindernoren's](https://github.com/eriklindernoren) pytorch implementation of yolov3, but can be easily modified to use any existing pytorch detector. Subscribes to '/image' and publishes detections to '/detections'. 
* openvslam_ros: wraps openvslam to subscribe to '/image' and publish to '/poses'. 
* quadricslam_ros: wraps the core quadricslam_online.py system. Subscribes to '/image', '/poses', '/detections'. 

## Installation 
### Dependencies

* [OpenVSLAM](https://github.com/xdspacelab/openvslam)
* [PyTorch-YOLOv3](https://github.com/eriklindernoren/PyTorch-YOLOv3) 
* [ROS2](https://index.ros.org/doc/ros2/Installation/) w/ packages: [cv_bridge](https://github.com/ros-perception/vision_opencv), [image_transport](https://github.com/ros-perception/image_common)
* Python3 w/ modules: pytorch, pillow, cv2, numpy, matplotlib

### Instructions

* Clone and build [QuadricSLAM](/README.md) with BUILD_PYTHON_WRAP set ON
    * Test QuadricSLAM is working with `python3 examples/python/simple_example.py`
* Clone and build OpenCV with contrib modules:

    ```
    git clone -b 3.4.10 https://github.com/opencv/opencv
    git clone -b 3.4.10 https://github.com/opencv/opencv_contrib
    cd opencv && mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DENABLE_CXX11=ON \
        -DBUILD_DOCS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_JASPER=OFF \
        -DBUILD_OPENEXR=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_TESTS=OFF \
        -DWITH_EIGEN=ON \
        -DWITH_FFMPEG=ON \
        -DWITH_OPENMP=ON \
        -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
        ..
    make -j4
    make install
    ```

* Follow the [OpenVSLAM installation guide](https://openvslam.readthedocs.io/en/master/installation.html): do not rebuild opencv 
    * When you have [calibrated your camera](https://github.com/ros-perception/image_pipeline) store the calibration as demonstrated in the openvslam examples (`openvslam/example/kitti/KITTI_mono_03.yaml`). 
    * Follow the [SLAM with UVC camera](https://openvslam.readthedocs.io/en/master/example.html) example
    * Test OpenVSLAM is working with a webcam using `./run_camera_slam -v /path/to/orb_vocab.dbow2 -c /path/to/webcam_config.yaml -n 0`
* Clone [PyTorch-YOLOv3](https://github.com/eriklindernoren/PyTorch-YOLOv3)
    * Ensure you have CUDA installed and a CUDA capable GPU
    * You can install pytorch by following [this guide](https://pytorch.org/get-started/locally/)
    * Install the requirements listed in requirements.txt
    * Follow the instructions to download the pretrained Yolov3 weights
    * Test the detector is working with `python3 detect.py --image_folder data/samples`
    * add /path/to/PyTorch-YOLOv3/ to PYTHONPATH
* Install ROS2 and additional packages:
    * Follow the [ROS installation guide](https://index.ros.org/doc/ros2/Installation/)
    * Install colcon to be able to build our own packages (`sudo apt-get install python3-colcon-common-extensions`)
    * Create a new ros2 workspace:
    
        ```
        $ cd ~
        $ mkdir -p ros_ws/src 
        ```

    * Clone the correct branch of [vision_opencv](https://github.com/ros-perception/vision_opencv) (for cv_bridge) and [image_common](https://github.com/ros-perception/image_common) (for image_transport) into the new workspace's /src folder. 

        ```
        $ cd ros_ws/src
        $ git clone -b ROSVERSION https://github.com/ros-perception/vision_opencv
        $ git clone -b ROSVERSION https://github.com/ros-perception/image_common
        ```

        where ROSVERSION is dashing/eloquent/foxy that you installed previously. 

    * Build the workspace `cd ~/ros_ws && colcon build --symlink-install`
    * Source the ROS2 overlay `. install/setup.bash` 
* Specify openvslam_SRC_DIR in [openvslam_ros/CMakeLists.txt](/ros/src/openvslam_ros/CMakeLists.txt) as the /path/to/openvslam 
* Navigate to /quadricslam/ros and build the ros packages with `colcon build --symlink-install`
* Source the overlay with `. install/setup.bash`

## Usage with USB Camera

Ensure you have correctly source the overlay containing cv_bridge and image_common, and our ROS overlay (ros/install/setup.bash) then:

* Start the webcam publisher: `ros2 run webcam_publisher run --ros-args -p width:=640 -p height:=480 -p n:=0`
* Start the object detector:  `ros2 run yolov3_ros run --ros-args -p weights:=/path/to/yolov3.weights -p config:=/path/to/yolov3.cfg -p classes:=/path/to/coco.names`
* Start OpenVSLAM tracking: `ros2 run openvslam_ros run -v /path/to/orb_vocab.dbow2 -c /path/to/webcam_config.yaml`
* Start QuadricSLAM mapping: `ros2 run quadricslam_ros run --config /path/to/quadricslam/config/webcam.yaml` 


## Common Issues

* If python struggles to import quadricslam modules from the ros node, ensure you have built the ros libraries with `colcon build --symlink-install`. 

* If you find that quadricslam_ros is not receiving any messages from /image, /detections and /poses, it might be because the TimeSynchronizer is giving up because the py_detector is taking too long to republish /detections. Ensure you have built pytorch with GPU support. 
