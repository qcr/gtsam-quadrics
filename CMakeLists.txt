project(gtsam_quadrics)
cmake_minimum_required(VERSION 3.0)
set(GTSAM_QUADRICS_VERSION 1.0.0)


# we want to use some C++11 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

###################################################################################
# set up options
option(BUILD_PYTHON_WRAP "Enable/Disable building of cython wrapper" ON)
set(PYTHON_VERSION "3" CACHE STRING "The version of python to build the cython wrapper for (or Default)")

###################################################################################
# Explicitly include GTSAM
if (BUILD_PYTHON_WRAP)
  set(GTSAM_BUILD_PYTHON ON CACHE BOOL "")
  set(GTSAM_CONFIG_TEMPLATE_PATH ${CMAKE_SOURCE_DIR}/gtsam/cmake)
endif()
set(GTSAM_BUILD_UNSTABLE OFF CACHE BOOL "")
add_subdirectory(./gtsam)

###################################################################################
# Add Python wrapping tools from gtwrap (used in GTSAM 4.1+)
# (we use the version of gtwrap explicitly bundled in the GTSAM submodule)
if(BUILD_PYTHON_WRAP)
  # The config.cmake for gtwrap seems to be buggy as hell at the moment; so
  # instead we manually hack in what we need from it here...
  # find_package(gtwrap REQUIRED)
  set(gtwrap_DIR ${CMAKE_SOURCE_DIR}/gtsam/wrap)

  list(APPEND CMAKE_MODULE_PATH ${gtwrap_DIR}/cmake)

  include(${gtwrap_DIR}/cmake/GtwrapUtils.cmake)
  include(${gtwrap_DIR}/cmake/PybindWrap.cmake)

  set(GTWRAP_PYTHON_PACKAGE_DIR ${gtwrap_DIR})
  set(PYBIND_WRAP_SCRIPT ${gtwrap_DIR}/scripts/pybind_wrap.py)
endif()

###################################################################################
# add gtsam_quadrics

# include headers, ensure that local folder is searched before library folders
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

# set source files
set(SOURCE_FILES
  ./gtsam_quadrics/base/Utilities.cpp
  ./gtsam_quadrics/geometry/ConstrainedDualQuadric.cpp
  ./gtsam_quadrics/geometry/AlignedBox2.cpp
  ./gtsam_quadrics/geometry/AlignedBox3.cpp
  ./gtsam_quadrics/geometry/BoundingBoxFactor.cpp
  ./gtsam_quadrics/geometry/QuadricAngleFactor.cpp
  ./gtsam_quadrics/geometry/QuadricCamera.cpp
  ./gtsam_quadrics/geometry/DualConic.cpp
  )

###################################################################################
## Build static library from common sources

set(CONVENIENCE_LIB_NAME "gtsam_quadrics")
add_library(${CONVENIENCE_LIB_NAME} ${SOURCE_FILES})
# set_target_properties(${CONVENIENCE_LIB_NAME} PROPERTIES PREFIX "")
target_link_libraries(${CONVENIENCE_LIB_NAME} gtsam)

###################################################################################
# install library and PACKAGEConfig.cmake
install(
  TARGETS ${CONVENIENCE_LIB_NAME}
  EXPORT GTSAM_QUADRICS-exports
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
GtsamMakeConfigFile(GTSAM_QUADRICS)
# list(APPEND GTSAM_EXPORTED_TARGETS ${CONVENIENCE_LIB_NAME})
# export(TARGETS ${GTSAM_EXPORTED_TARGETS} FILE GTSAM-exports.cmake)

file(GLOB gtsam_quadrics_base_headers "gtsam_quadrics/base/*.h")
install(FILES ${gtsam_quadrics_base_headers} DESTINATION include/gtsam_quadrics/base)
file(GLOB gtsam_quadrics_geometry_headers "gtsam_quadrics/geometry/*.h")
install(FILES ${gtsam_quadrics_geometry_headers} DESTINATION include/gtsam_quadrics/geometry)

###################################################################################
# Build tests (CMake tracks the dependecy to link with GTSAM through our project's static library)
gtsamAddTestsGlob("geometry" "gtsam_quadrics/geometry/tests/test*.cpp" "" "${CONVENIENCE_LIB_NAME}")
gtsamAddTestsGlob("geometry" "gtsam_quadrics/base/tests/test*.cpp" "" "${CONVENIENCE_LIB_NAME}")

###################################################################################
# Build example files (CMake tracks the dependecy to link with GTSAM through our project's static library)
# TODO fix broken examples!
# gtsamAddExamplesGlob("*.cpp" "" "${CONVENIENCE_LIB_NAME}")
# build examples if make examples
# https://stackoverflow.com/questions/17765618/cmake-add-subdirectories-to-custom-target-and-adding-custom-target-to-vs-solut
# TODO: pass ${CONVENIENCE_LIB_NAME} to examples/c++/CMakeLists.txt
# add_custom_target(examples) <-- already exists from GtsamTesting
# add_subdirectory(examples/c++)
# add_dependencies(examples examples_folder)

###################################################################################
# Build Python wrapper using gtwrap library for GTSAM 4.1+
if(BUILD_PYTHON_WRAP)
  pybind_wrap(
    ${PROJECT_NAME}_py
    ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/${PROJECT_NAME}.i
    "${PROJECT_NAME}.cpp"
    "${PROJECT_NAME}"
    "${PROJECT_NAME}"
    ""
    ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/${PROJECT_NAME}.tpl
    ${PROJECT_NAME}
    "${PROJECT_NAME}"
    ON
    )

  set_target_properties(${PROJECT_NAME}_py PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")
endif()

###################################################################################
# Build Doxygen documentation

# find doxygen
find_package(Doxygen)

# add subdirectory with custom make doc, doc_clean targets
if (DOXYGEN_FOUND)
  add_subdirectory(doc)
endif()
