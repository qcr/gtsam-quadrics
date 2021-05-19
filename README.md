# GTSAM Quadrics: quadric landmarks for GTSAM #

<!-- badges -->
[![Best of ACRV Repository](https://img.shields.io/badge/collection-best--of--acrv-%23a31b2a)](https://roboticvision.org/best-of-acrv)
[![License](https://img.shields.io/github/license/best-of-acrv/gtsam_quadrics)](./LICENSE.txt)

This repository contains an extension to the popular [GTSAM](https://github.com/borglab/gtsam) factor graph optimization library. We introduce constrained dual quadrics as GTSAM variables, and support the estimation of the quadric parameters using 2-D bounding box measurements. These tools are available for both c++ and Python and are designed to be used in conjunction with GTSAM. Also provided are a number of c++ and Python examples that demonstrate how quadric landmarks can be used in the SLAM context, alongside doxygen documentation and unit tests. 

<p align="center">
<img alt="RefineNet sample image on PASCAL VOC dataset" src=doc/quadricslam_still1.png width="400"/>
<img alt="RefineNet sample image on PASCAL VOC dataset" src=doc/quadricslam_still2.png width="400"/>
</p>

We expect this repository to be active and continually improved upon. If you have any feature requests or experience any bugs, don't hesitate to let us know. Our code is free to use, and licensed under BSD-3. We simply ask that you [cite our work](#citing-our-work) if you use QuadricSLAM in your own research. 

[![IMAGE ALT TEXT HERE](doc/quadricslam_video.png)](https://www.youtube.com/watch?v=n-j0DFDFSKU)



## Installation ## 

### Building and installation

After installing the required dependencies, build the core c++ gtsam_quadrics library:

```sh
# clone the repository 
$ git clone https://github.com/RoboticVisionOrg/gtsam_quadrics
$ cd gtsam_quadrics

# create build folder 
$ mkdir build
$ cd build

# (if you plan on using the python library)
$ cmake -DBUILD_PYTHON_WRAP=ON ..
# (otherwise, if you only want the gtsam_quadrics c++ library)
$ cmake -DBUILD_PYTHON_WRAP=OFF ..

# compile the library
$ make -j8

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

### Dependencies 

* g++ compiler (`sudo apt-get install build-essential`)
* cmake >= 3.0 (`sudo apt-get install cmake`) 
* boost >= 1.43 (`sudo apt-get install libboost-all-dev`)
* metis (`sudo apt-get install libmetis-dev`) <!-- in future, automatically get from gtsam/3rdparty, required when including gtsam/Symbol.h etc, maybe we just need to update some path? -->
* [gtsam](https://github.com/borglab/gtsam) <= 4.0.3

Requirements to build gtsam_quadrics python module:

* gtsam: build with GTSAM_INSTALL_CYTHON_TOOLBOX=ON, and included on PYTHONPATH
* cython: both `sudo apt-get install cython` and `pip3 install cython` required <!-- gtsam requisite --> <!-- maybe we can use one and update our CYTHON_PATH? --> <!-- gtsam only needs apt-get version -->
* python >= 3.0 <!-- gtsam requisite -->
* numpy <!-- gtsam requisite --> 

Optional requirements to build documentation:

* Doxygen (`sudo apt-get install doxygen`)
* epstopdf (`sudo apt-get install textlive-font-utils`)


### Notes on installation
**If you plan to use gtsam_quadrics in c++:** You can find the installed C++ headers using the cmake command `find_package(GTSAM_QUADRICS REQUIRED)` which will load `GTSAM_QUADRICS_INCLUDE_DIR`. The default header installation is `/usr/local/include/gtsam_quadrics/`, and by default library is installed to `/usr/local/lib/libgtsam_quadrics.so`. 

**If you plan to use gtsam_quadrics in python:** ensure you have build location (`/build/cython/gtsam_quadrics`) or the install location (default: `/usr/local/cython/gtsam_quadrics`) on your PYTHONPATH. If you plan to make changes to the gtsam_quadrics source code, I advice adding the build location instead to avoid having to install every time you recompile the library. Assuming you have followed the instructions above, we can automatically source the installed gtsam_quadrics library by explicitly adding the following line to your ~/.bashrc file. 

```sh
# add gtsam_quadrics to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:/usr/local/cython/gtsam_quadrics
```

<!-- ## Using QuadricSLAM

This library can be used to incorperate quadric landmarks into existing SLAM systems, or -->

<!-- Three types of users. 1. builds their own system from gtsam_quadrics -->
<!-- Three types of users. 2. modifies our scripts to add their own methods -->
<!-- Three types of users. 3. uses our scripts -->





## Planned developments
* High-level SLAM front-end (akin to ORBSLAM2)
* A BoundingBoxFactor error function robust to partially visible objects
* Faster and more robust data-association strategy 
* Easy webcam running (without ROS2) by using a python based odometry system
* Depth-based initialization for quadrics
* Semantic factors 

## Notes ##

### Adding Quadrics to gtsam::Values ###
When using the python interface, ConstrainedDualQuadrics can be added or retrieved from Values using the following. Since GTSAM 4.0 the python interface for Values manually specializes each type. In future we plan to derive the gtsam::Values class and add the insert/at methods for ConstrainedDualQuadric. 

```Python
quadric.addToValues(values, key)
quadric = quadricslam.ConstrainedDualQuadric.getFromValues(values, key)
```


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

Ensure that quadricslam is built with BUILD_PYTHON_WRAP set ON, the correct python version is used, and that the generated quadricslam.so shared library is on your PYTHONPATH. I.e, if you have installed quadricslam, that you have the following line in your ~/.bashrc

```
export PYTHONPATH=$PYTHONPATH:/usr/local/cython/quadricslam
```

## Citing our work ##
If you are using this library in academic work, please cite the [publication](https://ieeexplore.ieee.org/document/8440105):

L. Nicholson, M. Milford and N. Sünderhauf, "QuadricSLAM: Dual Quadrics From Object Detections as Landmarks in Object-Oriented SLAM," in IEEE Robotics and Automation Letters, vol. 4, no. 1, pp. 1-8, Jan. 2019, doi: 10.1109/LRA.2018.2866205. [PDF](https://arxiv.org/abs/1804.04011).

```bibtex
@article{nicholson2019,
  title={QuadricSLAM: Dual Quadrics From Object Detections as Landmarks in Object-Oriented SLAM},
  author={Nicholson, Lachlan and Milford, Michael and Sünderhauf, Niko},
  journal={IEEE Robotics and Automation Letters},
  year={2019},
}
```
