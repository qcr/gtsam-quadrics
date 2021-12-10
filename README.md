# GTSAM Quadrics: quadric landmarks for GTSAM

<!-- badges -->

[![Best of ACRV Repository](https://img.shields.io/badge/collection-best--of--acrv-%23a31b2a)](https://roboticvision.org/best-of-acrv)
[![License](https://img.shields.io/github/license/best-of-acrv/gtsam-quadrics)](./LICENSE.txt)

This repository contains an extension to the popular [GTSAM](https://github.com/borglab/gtsam) factor graph optimization library. We introduce constrained dual quadrics as GTSAM variables, and support the estimation of the quadric parameters using 2-D bounding box measurements. These tools are available for both C++ and Python and are designed to be used in conjunction with GTSAM. Also provided are a number of C++ and Python examples that demonstrate how quadric landmarks can be used in the SLAM context, alongside doxygen documentation and unit tests.

<p align="center">
<img alt="QuadricSLAM sample output image 1" src=https://github.com/best-of-acrv/gtsam-quadrics/raw/master/doc/quadricslam_still1.png width="400"/>
<img alt="QuadricSLAM sample output image 1" src=https://github.com/best-of-acrv/gtsam-quadrics/raw/master/doc/quadricslam_still2.png width="400"/>
</p>

We expect this repository to be active and continually improved upon. If you have any feature requests or experience any bugs, don't hesitate to let us know. Our code is free to use, and licensed under BSD-3. We simply ask that you [cite our work](#citing-our-work) if you use QuadricSLAM in your own research.

[![@youtube QuadricSLAM demonstration for RA-L](https://github.com/best-of-acrv/gtsam-quadrics/raw/master/doc/quadricslam_video.png)](https://www.youtube.com/watch?v=n-j0DFDFSKU)

## Installation

GTSAM Quadrics contains both C++ libraries, and Python wrappers for use in Python. We offer a number of different install methods, from single step methods to more involved depending on your desired use case.

Python wrappers can be installed via one of the following options:

1. [Through our Conda package](#conda): single command installs everything including system dependencies (recommended)
2. [Through our pip package](#pip): single command installs GTSAM and GTSAM Quadrics Python modules and Python dependences, but you take care of system dependencies
3. [Directly from source](#from-source): allows easy editing and extension of our code, but you take care of building and all dependencies

Or you can use the C++ libraries directly by:

1. [Building with CMake](#building-with-cmake): builds the library from scratch, and allows you to install into your system library paths

Please note that for all methods except the Conda method, you must have the following system dependencies installed beforehand:

- A C++ compiler: e.g. `sudo apt install build-essential`
- CMake >= 3.0: `sudo apt install cmake`
- Boost C++ libraries >= 1.43: `sudo apt install libboost-all-dev`
- METIS matrix library: `sudo apt install libmetis-dev` <!-- in future, automatically get from gtsam/3rdparty, required when including gtsam/Symbol.h etc, maybe we just need to update some path? -->

### Conda

TODO: actually make feedstock

The only requirement is that you have [Conda installed](https://conda.io/projects/conda/en/latest/user-guide/install/index.html) on your system. We provide Conda packages through [Conda Forge](https://conda-forge.org/), which recommends adding their channel globally with strict priority:

```
conda config --add channels conda-forge
conda config --set channel_priority strict
```

Once you have access to the `conda-forge` channel, GTSAM Quadrics is installed by running the following from inside a [Conda environment](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html):

```
conda install gtsam_quadrics
```

You can see a list of our Conda dependencies in [the feedstock recipe for GTSAM Quadrics](https://github.com/conda-forge/gtsam-quadrics-feedstock/blob/master/recipe/meta.yaml).

### Pip

TODO: actually add package to PyPI

First, install the system dependencies mentioned above have been installed. Then pre-built Python modules for both GTSAM Quadrics and GTSAM can be installed via:

```
pip install gtsam_quadrics
```

### From source

Installing from source is very similar to the `pip` method above, accept we install from a local copy. Ensure the system dependencies described above are installed.

Then clone the repository, and initialise the `gtsam` submodule:

```
git clone --recurse-submodules https://github.com/best-of-acrv/gtsam-quadrics
```

Enter the `gtsam_quadrics` directory, and simply install via `pip` (the build process will take a while):

```
pip install .
```

### Building with CMake

This process will build the library from scratch using CMake. It is very similiar to the "from source" method above, we just use CMake directly to build instead of invoking it as part of the `pip` install process.

Ensure required system dependencies are installed, then clone with the `gtsam` submodule initialised:

```
git clone --recurse-submodules https://github.com/best-of-acrv/gtsam-quadrics
```

Create an out-of-source build directory:

```
cd gtsam_quadrics
mkdir build
cd build
```

Run the configuration and generation CMake steps, optionally building the Python wrapper using the `BUILD_PYTHON_WRAP` variable:

```
cmake -DBUILD_PYTHON_WRAP=ON ..
```

Run the build step:

```
cmake --build . -j$(nproc)
```

Then optionally run any of the other supported targets as described below:

| **Target name** | **Description**                                |
| :-------------- | :--------------------------------------------- |
| check           | compile and run optional unit tests            |
| examples        | compiles the c++ examples                      |
| doc             | generates the doxygen documentation            |
| doc_clean       | removes the doxygen documentation              |
| install         | installs the gtsam_quadrics c++/python library |

_Note: documentation requires Doxygen (`sudo apt install doxygen`) and epstopdf (`sudo apt install texlive-font-utils`)_

For example, to install the library into system paths run:

```
cmake --build . --target install
```

TODO check these all still work???

## Using the GTSAM Quadrics and GTSAM Python APIs

GTSAM Quadrics and GTSAM can be used like native Python packages. Below are some examples to help get you started with using GTSAM Quadrics:

```python
# Note: at this stage you MUST import gtsam before importing gtsam_quadrics
import gtsam
import gtsam_quadrics
import numpy as np

# setup constants
pose_key = int(gtsam.symbol(ord('x'), 0))
quadric_key = int(gtsam.symbol(ord('q'), 5))

# create calibration
calibration = gtsam.Cal3_S2(525.0, 525.0, 0.0, 160.0, 120.0)

# create graph/values
graph = gtsam.NonlinearFactorGraph()
values = gtsam.Values()

# create noise model (SD=10)
bbox_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([10]*4, dtype=np.float))

# create quadric landmark (pose=eye(4), radii=[1,2,3])
initial_quadric = gtsam_quadrics.ConstrainedDualQuadric(gtsam.Pose3(), np.array([1.,2.,3.]))

# create bounding-box measurement (xmin,ymin,xmax,ymax)
bounds = gtsam_quadrics.AlignedBox2(15,12,25,50)

# create bounding-box factor
bbf = gtsam_quadrics.BoundingBoxFactor(bounds, calibration, pose_key, quadric_key, bbox_noise)

# add landmark to values
initial_quadric.addToValues(values, quadric_key)

# add bbf to graph
graph.add(bbf)


# get quadric estimate from values (assuming the values have changed)
quadric_estimate = gtsam_quadrics.ConstrainedDualQuadric.getFromValues(values, quadric_key)
```

## Planned developments

TODO check these are up-to-date

- High-level SLAM front-end (akin to ORBSLAM2)
- Support for expmap/logmap
- Tools to visualize and evaluate quadric landmarks

## Assorted notes and known issues

**How is the GTSAM dependency handled:** This repository includes GTSAM as a submodule, so you can see precisely which version we build against (currently it is post 4.1rc due to Python wrapper changes). The GTSAM Quadrics library is then built and linked against this copy of GTSAM to ensure compatibility. The Python module is also linked against the GTSAM Python module built from our submodule.

TODO need to check we handle this correctly
**If using GTSAM 4.0.3 or exponential-map rotations:** gtsam 4.0.3 moved to exponential map by default to parametrize rotations. The analytical derivatives we've calculated from this library are based on the cayley transform. Please either select cayley rotations in the gtsam CMakelists or use numerical derivatives (defined in boundingboxfactor.cpp).

**If you plan to use gtsam_quadrics in C++:** You can find the installed C++ headers using the cmake command `find_package(GTSAM_QUADRICS REQUIRED)` which will load `GTSAM_QUADRICS_INCLUDE_DIR`. The default header installation is `/usr/local/include/gtsam_quadrics/`, and by default library is installed to `/usr/local/lib/libgtsam_quadrics.so`.

**Adding Quadrics to `gtsam::Values`:** When using the python interface, ConstrainedDualQuadrics can be added or retrieved from Values using the following. Since GTSAM 4.0 the python interface for Values manually specializes each type. When supported by GTSAM, we plan to derive the gtsam::Values class and add the insert/at methods for ConstrainedDualQuadric.

```Python
quadric.addToValues(values, key)
quadric = gtsam_quadrics.ConstrainedDualQuadric.getFromValues(values, key)
```

TODO ensure this is still relevant?
**`AttributeError: module 'gtsam_quadrics' has no attribute 'ConstrainedDualQuadric'`:** If you import gtsam_quadrics and find it does not contain any attributes, or receive the above, ensure that gtsam_quadrics is built with BUILD_PYTHON_WRAP set ON, the correct python version is used, and that the generated gtsam_quadrics.so shared library is on your PYTHONPATH. I.e, if you have installed gtsam_quadrics, that you have the following line in your ~/.bashrc

```
export PYTHONPATH=$PYTHONPATH:/usr/local/cython/gtsam_quadrics
```

## Citing our work

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
