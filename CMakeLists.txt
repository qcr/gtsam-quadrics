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
	./quadricslam/base/Noise.cpp
	./quadricslam/base/TestClass.cpp
	./quadricslam/base/Jacobians.cpp
	./quadricslam/base/Utilities.cpp
	./quadricslam/geometry/ConstrainedDualQuadric.cpp
	./quadricslam/geometry/AlignedBox2.cpp
	./quadricslam/geometry/AlignedBox3.cpp
	./quadricslam/geometry/BoundingBoxFactor.cpp
	./quadricslam/geometry/QuadricCamera.cpp
	./quadricslam/geometry/DualConic.cpp
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
add_executable(expressions ./quadricslam/base/executables/testExpressions.cpp)
add_executable(poseJacobians ./quadricslam/base/executables/testPoseJacobians.cpp)

# link with gtsam (or recieve undefined refrences)
target_link_libraries(main gtsam;${CONVENIENCE_LIB_NAME})
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

###################################################################################
# Build Doxygen documentation 
find_package(Doxygen)
option(QUADRICSLAM_BUILD_DOCS "Enable/Disable building of doxygen docs" ${DOXYGEN_FOUND})
if (QUADRICSLAM_BUILD_DOCS)

	# quadricslam core subfolders
	set(quadricslam_doc_subdirs 
		quadricslam/base
		quadricslam/geometry
	)

	# set(doc_subdirs ${quadricslam_doc_subdirs})

	# From subfolders, build a list with whitespace separation of paths
	set(QUADRICSLAM_DOXYGEN_INPUT_PATHS "")
	foreach(dir ${quadricslam_doc_subdirs})
		set(QUADRICSLAM_DOXYGEN_INPUT_PATHS "${QUADRICSLAM_DOXYGEN_INPUT_PATHS} ${PROJECT_SOURCE_DIR}/${dir}")
	endforeach()

	# Generate Doxyfile
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile @ONLY)

	# Add target to actually build documentation as configured
	add_custom_target(doc
			${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc/
			COMMENT "Generating API documentation with Doxygen" VERBATIM
	)

	# Clean target
	add_custom_target(doc_clean
			COMMAND
					cmake -E remove_directory ${CMAKE_CURRENT_SOURCE_DIR}/doc/latex
			COMMAND
					cmake -E remove_directory ${CMAKE_CURRENT_SOURCE_DIR}/doc/html
			COMMENT "Removing Doxygen documentation"
	)

endif()
