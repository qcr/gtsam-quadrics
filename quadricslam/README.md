# QuadricSLAM: python front-end

QuadricSLAM is a python based front-end which utilizes our gtsam_quadrics back-end. The offline version of our system is intended to be run on datasets that provide ground truth camera positions and data-associations, the ability to determine which 2D object detection corrosponds to which 3D object; in other words, the dataset provides a source of odometry, 2D object bounding boxes, and instance level annotations. We inject noise into the camera odometry, initialize quadric landmarks using SVD, and optimizes the assembled factor graph and initial estimate using LevenbergMarquardt optimisation. 

The online version of our system is intended to be run live on a usb camera, providing you know the intrinsic calibration parameters. Behind the scenes we use isam2 optimisation for incremental factor graph optimisation, and we run a number of ROS2 nodes that collect camera images, calculate odometry measurements, and find 2D object detections. We provide a strategy for data-association and landmark initialization. 

We provide tools to interface with the SceneNet dataset, visualize quadrics, evaluate trajectories against ground truth, and interface OpenVSLAM / Yolov3 with ROS2. 

## QuadricSLAM: offline (dataset) ## 

If you wish to run the example python frontend on the SceneNetDataset, we require: 

* [pySceneNetRGBD](https://github.com/jmccormac/pySceneNetRGBD) cloned and built such that scenenet_pb2.py exists in root folder
* [SceneNetRGBD Data](https://robotvault.bitbucket.io/scenenet-rgbd.html) train and/or validation downloaded associated with protobuf files
* Google Protobuf (`pip3 install protobuf`)

After building the gtsam_quadrics library with python bindings and installing the required dependencies the offline system can be run using: `python3 quadricslam/quadricslam_offline.py --config quadricslam/config/scenenet.yaml`. We have provided config files for use with SceneNet and a simple Simulated dataset. In order to run the system on SceneNet, you will need to modify scenenet.yaml with the correct paths to the dataset. Additionally, you can modify the seed, noise levels, noise estimates and more. 

### QuadricSLAM: online (webcam) ### 

To run the online system from a webcam, we require:

* [OpenVSLAM](https://github.com/xdspacelab/openvslam)
* [PyTorch-YOLOv3](https://github.com/eriklindernoren/PyTorch-YOLOv3) 
* [ROS2](https://index.ros.org/doc/ros2/Installation/) w/ packages: [cv_bridge](https://github.com/ros-perception/vision_opencv), [image_transport](https://github.com/ros-perception/image_common)
* Python3 w/ modules: pytorch, pillow, cv2, numpy, matplotlib

Out of the box, we provide the ability to run QuadricSLAM from a webcam using [OpenVSLAM](https://github.com/xdspacelab/openvslam) for camera positions, [Yolov3](https://github.com/eriklindernoren/PyTorch-YOLOv3) for object detections, and [ROS2](https://index.ros.org/doc/ros2/Installation/) to tie everything together. We understand that installing ROS2 packages can be difficult for those new to ROS, and provide [detailed installation instructions here.](/ros/README.md)

With a source of images, camera positions and object detections on the topics `image`, `poses`, `detections` respectively, our online system can be run using: `ros2 run quadricslam_ros run --config /path/to/quadricslam/config/webcam.yaml`. 
