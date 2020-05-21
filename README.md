# README - QuadricSLAM #

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
* Add quadricslam/build/cython/ to PYTHONPATH or move

## build targets ##

* make
* make clean
* make check
* make examples
* make doc
* make doc_clean

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

Required to build Docs

* Doxygen (`sudo apt-get install doxygen`)

## Common Issues ##

If you attempt to build quadricslam and receive:

```
cython/quadricslam/quadricslam.pxd:1:0: 'gtsam/gtsam.pxd' not found
```

Ensure that gtsam is installed with the cython toolbox, and that it is on the PYTHONPATH. You can test this by attempting to import gtsam from within python. 
