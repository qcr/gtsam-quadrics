# QuadricSLAM: Quadric-based SLAM #

![System Performance Gif](https://s7.gifyu.com/images/performance_20_07_03_512.gif)

## Overview ##
This repository contains **gtsam_quadrics**, an extension to the popular [GTSAM](https://github.com/borglab/gtsam) factor graph optimization library which provides support for quadric landmarks constrained from boundingbox measurements typically produced by an object detector. We also provide support for using these extensions in python, alongside a number of c++ and python examples, doxygen documentation and unit tests. 

Also included in this repository is our **QuadricSLAM** system, able to run both incrementally (with isam2) and offline. The system is written in python and requires a feed images, object detections and odometry information and will estimate the current camera position and quadric map. We provide a ROS2 interface in order to use [OpenVSLAM](https://arxiv.org/abs/1910.01122) (C++) for localization and [Yolov3](https://pjreddie.com/media/files/papers/YOLOv3.pdf) (Python/PyTorch) for object detections. 

If you use QuadricSLAM in your research, please cite our paper with the BibTeX entry shown below. 

## Related Paper ##

L. Nicholson, M. Milford and N. Sünderhauf, "QuadricSLAM: Dual Quadrics From Object Detections as Landmarks in Object-Oriented SLAM," in IEEE Robotics and Automation Letters, vol. 4, no. 1, pp. 1-8, Jan. 2019, doi: 10.1109/LRA.2018.2866205. [PDF](https://arxiv.org/abs/1804.04011).

If you are using this library in academic work, please cite the [publication](https://ieeexplore.ieee.org/document/8440105):

    @ARTICLE{8440105,
      author={L. {Nicholson} and M. {Milford} and N. {Sünderhauf}},
      journal={IEEE Robotics and Automation Letters}, 
      title={QuadricSLAM: Dual Quadrics From Object Detections as Landmarks in Object-Oriented SLAM}, 
      year={2019},
      volume={4},
      number={1},
      pages={1-8},
    }



## Requirements ## 

### gtsam_quadrics: back-end library ###
* g++ compiler (`sudo apt-get install build-essential`)
* cmake >= 3.0 (`sudo apt-get install cmake`) 
* boost >= 1.43 (`sudo apt-get install libboost-all-dev`)
* metis (`sudo apt-get install libmetis-dev`) <!-- in future, automatically get from gtsam/3rdparty, required when including gtsam/Symbol.h etc, maybe we just need to update some path? -->
* [gtsam](https://github.com/borglab/gtsam)

Optional requirements to build documentation:

* Doxygen (`sudo apt-get install doxygen`)
* epstopdf (`sudo apt-get install textlive-font-utils`)

Requirements to build gtsam_quadrics python module:

* gtsam: build with GTSAM_INSTALL_CYTHON_TOOLBOX=ON, and included on PYTHONPATH
* cython: both `sudo apt-get install cython` and `pip3 install cython` required <!-- gtsam requisite --> <!-- maybe we can use one and update our CYTHON_PATH? --> <!-- gtsam only needs apt-get version -->
* python >= 3.0 <!-- gtsam requisite -->
* numpy <!-- gtsam requisite --> 

### QuadricSLAM: python SLAM system ###
* OpenCV2 (`pip3 install opencv-python`)
* Matplotlib (`pip3 install matplotlib`)

Required to test on SceneNetRGBD:

* [pySceneNetRGBD](https://github.com/jmccormac/pySceneNetRGBD)
* [SceneNetRGBD Data](https://robotvault.bitbucket.io/scenenet-rgbd.html)
* [ShapeNet Dataset](https://www.shapenet.org/): optional, required to initialize quadrics using true object dimensions
* Google Protobuf (`pip3 install protobuf`)

Required to test on USB Webcam: 

* [OpenVSLAM](https://github.com/xdspacelab/openvslam)
* [PyTorch-YOLOv3](https://github.com/eriklindernoren/PyTorch-YOLOv3)
* [ROS2](https://index.ros.org/doc/ros2/Installation/) w/ packages: [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2), [image_transport](https://github.com/ros-perception/image_common/tree/ros2)
* Python3 w/ modules: pytorch, pillow, cv2, numpy, matplotlib, pyyaml







## QuadricSLAM: quick start ## 

After installing all required dependencies, build the core c++ gtsam_quadrics library:

```sh
# clone the repository 
$ git clone https://github.com/RoboticVisionOrg/quadricslam
$ cd quadricslam

# create build folder 
$ mkdir build
$ cd build

# (if you plan on using QuadricSLAM or want the python library)
$ cmake -DBUILD_PYTHON_WRAP=ON ..
# (otherwise, if you only want the gtsam_quadrics c++ library)
$ cmake -DBUILD_PYTHON_WRAP=OFF ..

# optional: run the c++ unit tests
$ make check 

# optional: install the c++ and/or python library 
$ make install
```

The following table summarizes the available targets 
| **Command**    | **Description**                                |
| :---------     |   :-----------------------------------------   |
| make check     | compile and run optional unit tests            | 
| make examples  | compiles the c++ examples                      | 
| make doc       | generates the doxygen documentation            | 
| make doc_clean | removes the doxygen documentation              | 
| make install   | installs the gtsam_quadrics c++/python library | 

If you plan to use gtsam_quadrics in python, ensure you have build location (/build/cython/gtsam_quadrics) or the install location (default: /usr/local/cython/gtsam_quadrics) on your PYTHONPATH. Assuming you have followed the instructions above, we can austomatically source the gtsam_quadrics library by explicitly adding the following line to your ~/.bashrc file. 

```sh
# add gtsam_quadrics to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:/usr/local/cython/gtsam_quadrics
```

If you plan to make changes to the gtsam_quadrics source code, it can be easier to add the build location so you don't have to "make install" each time you recompile. 



<!-- The following list outlines the many ways to use our software: 
* Run QuadricSLAM offline on a dataset: 
* Run QuadricSLAM online in python:
* Run QuadricSLAM online using ROS2:
* Create your own system using quadric landmarks (C++):
* Create your own system using quadric landmarks (Python): -->


## QuadricSLAM: offline (dataset) ## 

run `python3 examples/python/example_frontend/system.py --config examples/python/example_frontend/SceneNetConfig.ini`

## QuadricSLAM: online (dataset) ## 

run `python3 examples/python/example_frontend/system.py --config examples/python/example_frontend/SceneNetConfig.ini`

## QuadricSLAM: online (webcam) ## 

run `python3 examples/python/example_frontend/system.py --config examples/python/example_frontend/SceneNetConfig.ini`

## gtsam_quadrics: create your own SLAM system using quadric landmarks (C++) ## 

After building and installing gtsam_quadrics, you can find the c++ headers and library using `find_package(QUADRICSLAM REQUIRED)`

## gtsam_quadrics: create your own SLAM system using quadric landmarks (Python) ## 






### Bounding Box Factor ###
We provide an implementation of a simple error function between quadric landmark and camera pose, although we plan to release the error function described in the paper in future. The key difference is that the simple error function calculates the bounds of the dual conic with no consideration for the image dimensions. Measurements where the objects bounds extend beyond the image boundaries generate a significant error even if the quadric is correctly fitting the object. In practice this means that the noise estimate for the boundingboxfactors should be overestimated. 


## Notes ##

When using the python interface, ConstrainedDualQuadrics can be added or retrieved from Values using:
```Python
    quadric.addToValues(values, key)
    quadric = quadricslam.ConstrainedDualQuadric.getFromValues(values, key)
```
Since GTSAM 4.0 the python interface for Values manually specializes each type, and we have not yet found a clean way to add our custom Quadric classes to the gtsam::Values object in python. In future we plan to derive the gtsam::Values class and add the insert/at methods for ConstrainedDualQuadric, however, the class is currently not polymorphic. 

## Common Issues ##

If you attempt to build quadricslam and receive:

```
cython/quadricslam/quadricslam.pxd:1:0: 'gtsam/gtsam.pxd' not found
```

Ensure that gtsam is installed with the cython toolbox enabled, and that it is on the PYTHONPATH. You can test this by attempting to import gtsam from within python. By default, gtsam is installed to /usr/local/cython. 

If you import quadricslam and find it does not contain any attributes, or recieve:

```
AttributeError: module 'quadricslam' has no attribute 'ConstrainedDualQuadric'
```

Ensure that quadricslam is built with QSLAM_BUILD_PYTHON_WRAP set ON, the correct python version is used, and that the generated quadricslam.so shared library is on your PYTHONPATH. I.e, if you have installed quadricslam, that you have the following line in your ~/.bashrc

```
export PYTHONPATH=$PYTHONPATH:/usr/local/cython/quadricslam
```
