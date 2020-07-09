cmake_minimum_required(VERSION 3.5)
project(openvslam_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# openvslam 
set(openvslam_SRC_DIR "/home/lachness/git_ws/openvslam")
set(openvslam_INCLUDE_DIRS "/usr/local/include/openvslam")
include_directories(${openvslam_INCLUDE_DIRS})

# thread library
find_package(Threads REQUIRED)

# OpenVSLAM
find_library(OpenVSLAM_LIB openvslam HINTS ${OpenVSLAM_LIB_DIR})

# OpenCV
find_package(OpenCV 3.3.1 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 4.0)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV >= 3.3.1 not found")
    endif()
endif()
message(STATUS "Use OpenCV ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# spdlog
set(spdlog_INCLUDE_DIR ${openvslam_INCLUDE_DIRS}/3rd/spdlog/include)
include_directories(${spdlog_INCLUDE_DIR})

# popl
set(popl_INCLUDE_DIR ${openvslam_SRC_DIR}/3rd/popl/include)
include_directories(${popl_INCLUDE_DIR})

# json
set(json_INCLUDE_DIR ${openvslam_INCLUDE_DIRS}/3rd/json/include)
include_directories(${json_INCLUDE_DIR})



# pangolin viewer
set(USE_PANGOLIN_VIEWER ON CACHE BOOL "Enable Pangolin Viewer")
if(USE_PANGOLIN_VIEWER)
    set(Pangolin_Viewer_INCLUDE_DIR ${openvslam_SRC_DIR}/src)
    include_directories(${Pangolin_Viewer_INCLUDE_DIR})
    find_library(Pangolin_Viewer_LIB pangolin_viewer HINTS ${openvslam_SRC_DIR}/build/lib)
    find_package(Pangolin REQUIRED)
endif()

# socket publisher
set(USE_SOCKET_PUBLISHER OFF CACHE BOOL "Enable Socket Publisher")
if(USE_SOCKET_PUBLISHER)
    find_package(sioclient REQUIRED)
    find_package(Protobuf REQUIRED)
    find_library(Socket_Publisher_LIB socket_publisher)
    if(NOT PROTOBUF_PROTOC_EXECUTABLE)
        message(FATAL_ERROR "Could not find protoc executable (PROTOBUF_PROTOC_EXECUTABLE)")
    endif()
    message(STATUS "Found protoc executable: ${PROTOBUF_PROTOC_EXECUTABLE}")
    include_directories(${SIOCLIENT_INCLUDE_DIR} ${PROTOBUF_INCLUDE_DIRS})
endif()

# gtsam
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})


# BoW Framework
set(BOW_FRAMEWORK "DBoW2" CACHE STRING "DBoW2 or FBoW")
set_property(CACHE BOW_FRAMEWORK PROPERTY STRINGS "DBoW2" "FBoW")

if(BOW_FRAMEWORK MATCHES "DBoW2")
    find_package(DBoW2 REQUIRED)
    set(BOW_INCLUDE_DIRS ${DBoW2_INCLUDE_DIRS})
    set(BOW_LIBS ${DBoW2_LIBS})
    add_definitions(-DUSE_DBOW2)
elseif(BOW_FRAMEWORK MATCHES "FBoW")
    find_package(fbow REQUIRED)
    set(BOW_INCLUDE_DIRS ${fbow_INCLUDE_DIRS})
    set(BOW_LIBS ${fbow_LIBS})
else()
    message(FATAL_ERROR "Invalid BoW framework: ${BOW_FRAMEWORK}")
endif()
include_directories(${BOW_INCLUDE_DIRS})
message(STATUS "BoW framework: ${BOW_FRAMEWORK} (found in ${BOW_INCLUDE_DIRS})")



# build executable target 
set(EXEC_NAME "run")
add_executable(${EXEC_NAME} src/run_slam.cpp)



# pangolin_viewer is used on a priority basis
if(USE_PANGOLIN_VIEWER)
    # set macro flag
    target_compile_definitions(${EXEC_NAME} PRIVATE -DUSE_PANGOLIN_VIEWER)
    # link viewer
    target_link_libraries(${EXEC_NAME} PRIVATE ${Pangolin_Viewer_LIB} ${Pangolin_LIBRARIES})
elseif(USE_SOCKET_PUBLISHER)
    # set macro flag
    target_compile_definitions(${EXEC_NAME} PRIVATE -DUSE_SOCKET_PUBLISHER)
    # link viewer
    target_link_libraries(${EXEC_NAME} PRIVATE ${Socket_Publisher_LIB} ${SIOCLIENT_LIBRARY} ${PROTOBUF_LIBRARIES})
elseif(USE_SOCKET_PUBLISHER)
    # set macro flag
    target_compile_definitions(${EXEC_NAME} PRIVATE -DUSE_SOCKET_PUBLISHER)
    # link viewer
    target_link_libraries(${EXEC_NAME} PRIVATE ${Socket_Publisher_LIB} ${SIOCLIENT_LIBRARY} ${PROTOBUF_LIBRARIES})
endif()

# setup stack trace logger
if(USE_STACK_TRACE_LOGGER)
    target_compile_definitions(${EXEC_NAME} PRIVATE -DUSE_STACK_TRACE_LOGGER)
    target_link_libraries(${EXEC_NAME} PRIVATE ${GLOG_LIBRARIES})
endif()

# setup google-perftools
if(USE_GOOGLE_PERFTOOLS)
    target_compile_definitions(${EXEC_NAME} PRIVATE -DUSE_GOOGLE_PERFTOOLS)
    target_link_libraries(${EXEC_NAME} PRIVATE ${GPERFTOOLS_LIBRARIES})
endif()

# build and install executable 
target_include_directories(${EXEC_NAME} PRIVATE ${openvslam_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIRS})
target_link_libraries(${EXEC_NAME} PRIVATE ${OpenVSLAM_LIB} ${rclcpp_LIBRARIES} ${OpenCV_LIBRARIES} ${GTSAM_LIBRARIES} gtsam)
ament_target_dependencies(${EXEC_NAME} PUBLIC rclcpp cv_bridge image_transport)
install(TARGETS ${EXEC_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
