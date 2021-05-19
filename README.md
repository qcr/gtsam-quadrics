# QuadricSLAM: Object-based localization and mapping #

This repository contains an extension to the popular [GTSAM](https://github.com/borglab/gtsam) factor graph optimization library. We introduce constrained dual quadrics as GTSAM variables, and support the estimation of the quadric parameters using 2-D bounding box measurements. These tools are available for both c++ and Python and are designed to be used in conjunction with GTSAM. Also provided are a number of c++ and Python examples that demonstrate how quadric landmarks can be used in the SLAM context, alongside doxygen documentation and unit tests. 

<!-- add opening image -->

We expect this repository to be active and continually improved upon. If you have any feature requests or experience any bugs, don't hesitate to let us know. Our code is free to use, and licensed under BSD-3. We simply ask that you [cite our work](#citing-our-work) if you use QuadricSLAM in your own research. 

<!-- add opening video -->

<!-- ## Installing QuadricSLAM -->

<!-- conda, pip (python), source (python or c++) -->
<!-- add different installation options -->


<!-- ## Limitations 
Although the effectiveness of quadric-based SLAM can be demonstrated to perform quite well under certain conditions, data-association remains a key limitation to the robustness of our system. The offline version requires that a dataset provides instance level 2D annotations, and the online system uses cv2 object trackers for data-association. To make it easier for the data-association strategy when working from a webcam, we have filtered out a subset of measurements corrosponding to certain object classes (i.e, cup, bowl, mouse). What this means is that if this filtering is turned off, the current data-association strategy will often fail to associate measurements correctly leading to duplicate quadric landmarks. 

Similarly, for our front-end, quadric initialization could also use some improvement, as listed in our [future work section](#planned-developments). Currently SVD is not very accurate unless used over a large baseline or with a high number of measurements. When data-association is accurate, the system performs well even with poor initialization. However, if a landmark only receives a few measurements after initialization, as is the case when data-association sends the measurements to a different landmark, the quadric will remain frozen and innacurately represent the objects bounds. We plan to use depth information or semantic priors to address this in the future.

The current back-end implementation of the BoundingBoxFactor uses a simple error function with analytical jacobians. Compared to the error function described in our paper, the simple error function calculates the bounds of the dual conic with no consideration for the image dimensions and therefore doesn't handle partially visible objects well. In practice, large objects that are often partially visible and will have their quadric bounds shrunk. To remidy this, the noise estimate for the boundingboxfactors should be overestimated. We plan to release the more advanced error function with analytical jacobians in the future.  -->
<!-- this error function will generate significant errors for measurements where the objects bounds extend beyond the image boundaries.  -->

## Installation from source ## 

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

### Building and installation

After installing the required dependencies, build the core c++ gtsam_quadrics library:

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
