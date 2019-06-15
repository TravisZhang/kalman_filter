cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter)

## Compile as C++17
add_compile_options(-std=c++17)

# Add Eigen3 support
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Add local libraries
include_directories (/usr/local/include)
link_directories (/usr/local/lib)

# Find yaml package
#file(GLOB yaml_sources "third_party/yaml-cpp/src/[a-zA-Z]*.cpp")

## Declare a C++ library
add_library(kalman_filter SHARED
  kalman_filter.cpp
  # ${yaml_sources}
)
# target_link_libraries(motion_control
#   yaml-cpp 
# )

target_include_directories(kalman_filter
PUBLIC
    ${kalman_filter_SOURCE_DIR} ${EIGEN3_INCLUDE_DIRS}
)
# set_target_properties(kalman_filter PROPERTIES LINKER_LANGUAGE CXX COMPILE_FLAGS "-fPIC")


## Add executable
add_executable (main main.cpp)
target_link_libraries (main kalman_filter)

