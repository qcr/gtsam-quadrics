# README #

This README documents dependencies and setup procedure

## Setup ##

Summary:
* Clone repository 
* Install dependencies 
* Run ./setup.py which will:
  * build and install gtsam_quadrics
  * select the correct python version
* Run tests with "make check" inside build

## Dependencies ##
### Core
* g++ compilter (sudo apt-get install build-essential)
* cmake (sudo apt-get install cmake)

### gtsam4
* numpy (sudo apt-get install python-numpy)
* Intel MKL (*optional*)
* Intel TBB (*optional*)
* Intel OpenMP (*optional*)
* cython (*for cython wrapper*) (sudo apt-get install cython)
* matlab (*for matlab wrapper*)

### gtsam_quadrics
* eigen3 > 2.91 (sudo apt-get install libeigen3-dev) (** or use gtsam eigen **)
* metis (sudo apt-get install libmetis-dev)

