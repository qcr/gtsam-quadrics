# README - QuadricSLAM #

## Description ##
At it's core, quadricslam extends gtsam to provide support for optimizing quadric landmarks. We introduce constrained quadric landmarks on a manifold and the ability to optimize these landmarks when seen from multiple views using boundingbox measurements. We also provide a python interface and a number of c++ and python examples, including an [example python front-end](examples/python/README.md) which can load and run sequences from the SceneNetRGBD dataset (using ground truth data-association). 

**Related Paper:**

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


### Bounding Box Factor ###
We provide an implementation of a simple error function between quadric landmark and camera pose, although we plan to release the error function described in the paper in future. The key difference is that the simple error function calculates the bounds of the dual conic with no consideration for the image dimensions. Measurements where the objects bounds extend beyond the image boundaries generate a significant error even if the quadric is correctly fitting the object. In practice this means that the noise estimate for the boundingboxfactors should be overestimated. 

## Quick Start ##

To build the c++ core library:

```sh
#!bash
$ mkdir build
$ cd build
$ cmake ..
$ make check (optional, runs unit tests)
$ make install
```

To enable the python interface:

* Enable QSLAM_BUILD_PYTHON_WRAP in the CMakeLists.txt
* Repeat the steps above to build the library
* Add /build/cython/gtsam_quadrics to PYTHONPATH or move to a location on your path

The provided tests can be run using `make check`

The provided examples can be compiled using `make examples`

The doxygen generated documentation can be build using `make doc` and removed with `make doc_clean`

The headers / library / python module can be installed using `make install` and removed with `make uninstall`

## Dependencies ##
Core C++ 

* g++ compiler (`sudo apt-get install build-essential`)
* cmake (`sudo apt-get install cmake`) 
* boost (`sudo apt-get install libboost-all-dev`)
* metis (`sudo apt-get install libmetis-dev`) <!-- in future, automatically get from gtsam/3rdparty, required when including gtsam/Symbol.h etc, maybe we just need to update some path? -->
* gtsam 
  * Intel MKL (*optional*)
  * Intel TBB (*optional*)
  * Intel OpenMP (*optional*)

Python Wrapper

* Gtsam Cython toolbox (included on PYTHONPATH and using the same python version)
* cython (`sudo apt-get install cython`, `pip3 install cython`) <!-- gtsam requisite --> <!-- maybe we can use one and update our CYTHON_PATH? --> <!-- gtsam only needs apt-get version -->
* python >= 3.0 <!-- gtsam requisite -->
* numpy <!-- gtsam requisite --> 

Python Front End

* OpenCV2 (`pip3 install opencv-python`)
* Matplotlib

Required to build Docs

* Doxygen (`sudo apt-get install doxygen`)
* epstopdf (`sudo apt-get install textlive-font-utils`)

Required to test the example_frontend on SceneNetRGBD:

* [pySceneNetRGBD](https://github.com/jmccormac/pySceneNetRGBD)
* [SceneNetRGBD Data](https://robotvault.bitbucket.io/scenenet-rgbd.html)
* [ShapeNet Dataset](https://www.shapenet.org/) (optional, required to initialize quadrics using true object dimensions)
* Google Protobuf (`pip3 install protobuf`)

## Using QuadricSLAM ##

If you wish to use this package in your own project, you can install the c++ headers and shared library using `make install`. By default the headers and library are installed to /usr/local/include and /usr/local/lib respectively. 

The python interface can be used, after building, by adding /build/cython/quadricslam to the PYTHONPATH. Alternatively, `make install` will install the python module, by default, to /usr/local/cython/quadricslam. Ensure you have this folder on your PYTHONPATH before attempting to import quadricslam. 

## Notes ##

When using the python interface, ConstrainedDualQuadrics can be added or retrieved from Values using:
```Python
    quadric.addToValues(values, key)
    quadric = quadricslam.ConstrainedDualQuadric.getFromValues(values, key)
```

Similarly, the boundingbox factor can be used with:
```Python
    bbf.addToGraph(graph)
    bbf = BoundingBoxFactor.getFromGraph(graph, index)
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

Ensure that quadricslam is built with QSLAM_BUILD_PYTHON_WRAP set ON, the correct python version is used, and that the generated quadricslam.so shared library is on your PYTHONPATH. I.e, if you have installed quadricslam, that you have the following line in your ~/.bashrc

```
export PYTHONPATH=$PYTHONPATH:/usr/local/cython/quadricslam
```
