project(quadricslam)
cmake_minimum_required(VERSION 3.0)

# we want to use some C++11 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

###################################################################################
# find GTSAM and GTSAM CMake tools
find_package(GTSAM REQUIRED)
find_package(GTSAMCMakeTools)

# use python 3.5, must be called before including gtsamcythonwrap
set(GTSAM_PYTHON_VERSION 3.5)

# include gtsam cmake tools
include(GtsamBuildTypes) # Load build type flags and default to Debug mode
include(GtsamTesting)    # Easy functions for creating unit tests and scripts
include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation
include(GtsamCythonWrap) # Automatic MATLAB wrapper generation

###################################################################################
# add quadricslam

# include headers, ensure that local folder is searched before library folders
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

# set source files
set(SOURCE_FILES
	./quadricslam/base/Evaluation.cpp
	./quadricslam/base/Noise.cpp
	./quadricslam/base/Dataset.cpp
	./quadricslam/base/TestClass.cpp
	./quadricslam/base/Jacobians.cpp
	./quadricslam/geometry/ConstrainedDualQuadric.cpp
	./quadricslam/geometry/AlignedBox2.cpp
	./quadricslam/geometry/BoundingBoxFactor.cpp
	./quadricslam/geometry/QuadricCamera.cpp
	./quadricslam/geometry/DualConic.cpp
	./quadricslam/containers/Trajectory.cpp
	./quadricslam/containers/Odometry.cpp
	./quadricslam/containers/QuadricMap.cpp
	./quadricslam/containers/Detections.cpp
)

###################################################################################
## Build static library from common sources
set(CONVENIENCE_LIB_NAME "cquadrics")
add_library(${CONVENIENCE_LIB_NAME} SHARED ${SOURCE_FILES})
set_target_properties(${CONVENIENCE_LIB_NAME} PROPERTIES PREFIX "")
target_link_libraries(${CONVENIENCE_LIB_NAME} gtsam)

# Install library
# install(TARGETS ${CONVENIENCE_LIB_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

###################################################################################
## Build executables and link to our shared library. NOTE: can't have two mains in source_files
# compile .exe files
add_executable(main ./quadricslam/main.cpp)
add_executable(runsystem ./quadricslam/base/System.cpp)
add_executable(expressions ./quadricslam/base/executables/testExpressions.cpp)
add_executable(poseJacobians ./quadricslam/base/executables/testPoseJacobians.cpp)

# link with gtsam (or recieve undefined refrences)
target_link_libraries(main gtsam;${CONVENIENCE_LIB_NAME})
target_link_libraries(runsystem gtsam;${CONVENIENCE_LIB_NAME})
target_link_libraries(expressions gtsam;${CONVENIENCE_LIB_NAME})
target_link_libraries(poseJacobians gtsam;${CONVENIENCE_LIB_NAME})


###################################################################################
# Build tests (CMake tracks the dependecy to link with GTSAM through our project's static library)
gtsamAddTestsGlob("geometry" "quadricslam/geometry/tests/test*.cpp" "" "${CONVENIENCE_LIB_NAME}")
gtsamAddTestsGlob("geometry" "quadricslam/base/tests/test*.cpp" "" "${CONVENIENCE_LIB_NAME}")

###################################################################################
# Build example files (CMake tracks the dependecy to link with GTSAM through our project's static library)
# gtsamAddExamplesGlob("*.cpp" "" "${CONVENIENCE_LIB_NAME}")

###################################################################################
# Build MATLAB wrapper (CMake tracks the dependecy to link with GTSAM through our project's static library)
# wrap_and_install_library("example.h" "${CONVENIENCE_LIB_NAME}" "" "")

###################################################################################
# Build Cython wrapper (CMake tracks the dependecy to link with GTSAM through our project's static library)
include_directories(${GTSAM_EIGENCY_INSTALL_PATH})
file(WRITE "./build/cython/quadricslam/__init__.py" "")
wrap_and_install_library_cython("./quadricslam.h" # interface_header path
								"from gtsam.gtsam cimport *" # extra imports for cython pxd
								"." # install_path
								"gtsam;${CONVENIENCE_LIB_NAME}" # libraries to link with
								"wrap;gtsam;${CONVENIENCE_LIB_NAME}" # dependencies)
)


