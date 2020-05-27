# find gtsam for eigen 
find_package(GTSAM REQUIRED)

# include project headers
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

# compile .exe files, exclude from all and build only if make examples
add_executable(exampleWithNoise EXCLUDE_FROM_ALL exampleWithNoise.cpp)
add_executable(simpleExample EXCLUDE_FROM_ALL simpleExample.cpp)

# link with gtsam (or recieve undefined refrences)
target_link_libraries(exampleWithNoise gtsam;${CONVENIENCE_LIB_NAME})
target_link_libraries(simpleExample gtsam;${CONVENIENCE_LIB_NAME})

# adding examples_folder target to list example dependencies here
# as opposed to listing under "make examples" in root CMakeLists
add_custom_target(examples_folder)
add_dependencies(examples_folder exampleWithNoise;simpleExample)