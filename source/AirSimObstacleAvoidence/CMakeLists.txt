cmake_minimum_required(VERSION 3.10)
project(AirSimObstacleAvoidence)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(AIRSIM_ROOT "/home/happy-coding/AirSim")
set(AIRSIM_BUILD_LIB "${AIRSIM_ROOT}/build/output/lib")

find_package(OpenCV REQUIRED)

include_directories(
        "${AIRSIM_ROOT}/AirLib/include"
        "${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include"
        ${OpenCV_INCLUDE_DIRS}
        "/usr/include/eigen3"
)

add_executable(AirSimObstacleAvoidence src/main.cpp)

target_link_libraries(AirSimObstacleAvoidence
        ${OpenCV_LIBRARIES}
        "${AIRSIM_BUILD_LIB}/libAirLib.a"
        "${AIRSIM_BUILD_LIB}/librpc.a"
        pthread
)