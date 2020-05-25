# README - QuadricSLAM #

## Description ##
At it's core, quadricslam extends gtsam to provide support for optimizing quadric landmarks. We introduce constrained quadric landmarks on a manifold and the ability to optimize these landmarks when seen from multiple views using boundingbox measurements. We also provide a python interface and a number of c++ and python examples, including an [example python front-end](examples/python/README.md) which can load and run sequences from the SceneNetRGBD dataset (using ground truth data-association). 

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
* Add quadricslam/build/cython/ to PYTHONPATH or move to a location on your path

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

## Tests ##

The provided tests can be run using `make check`

## Building C++ Examples ##

The provided examples can be compiled using `make examples`

## Generating Documentation ##

The doxygen generated documentation can be build using `make doc` and removed with `make doc_clean`

## Adding Variables and Factors to Graph / Values in Python ##

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
