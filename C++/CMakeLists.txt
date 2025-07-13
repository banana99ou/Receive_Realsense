cmake_minimum_required(VERSION 3.15)
project(RealSenseApp)

# Tell CMake where to find the RealSense SDK
find_package(realsense2 REQUIRED)

add_executable(RealSenseApp main.cpp)
target_link_libraries(RealSenseApp PRIVATE realsense2::realsense2)