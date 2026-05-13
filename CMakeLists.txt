cmake_minimum_required(VERSION 3.5.0)
project(carint_touch)

# Set default build type if not specified
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Use modern C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(CapnProto REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(Franka REQUIRED)

# libfranka
set(CMAKE_PREFIX_PATH "/home/john/libfranka_v0.9")
find_package(Franka REQUIRED)


# include Eigen3
include_directories(${EIGEN3_INCLUDE_DIR})


# compile the executables
add_executable(caring_touch
            caring_touch.cpp
            utils/pose_mover.cpp
            utils/examples_common.cpp)

# Link libraries
target_link_libraries(caring_touch
    Franka::Franka
)