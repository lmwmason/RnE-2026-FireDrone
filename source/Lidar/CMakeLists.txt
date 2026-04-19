cmake_minimum_required(VERSION 3.20)
project(RnELidar)

set(CMAKE_CXX_STANDARD 14)

include_directories(libs/rplidar_sdk/sdk/include)
include_directories(libs/rplidar_sdk/sdk/src)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB SDK_CORE_SOURCES "libs/rplidar_sdk/sdk/src/*.cpp")
file(GLOB SDK_UNPACKER_SOURCES "libs/rplidar_sdk/sdk/src/dataunpacker/*.cpp")
file(GLOB SDK_UNPACKER_HANDLER_SOURCES "libs/rplidar_sdk/sdk/src/dataunpacker/unpacker/*.cpp")
file(GLOB SDK_HAL_SOURCES "libs/rplidar_sdk/sdk/src/hal/*.cpp")
file(GLOB SDK_ARCH_SOURCES "libs/rplidar_sdk/sdk/src/arch/linux/*.cpp")

set(SDK_SOURCES
        ${SDK_CORE_SOURCES}
        ${SDK_UNPACKER_SOURCES}
        ${SDK_UNPACKER_HANDLER_SOURCES}
        ${SDK_HAL_SOURCES}
        ${SDK_ARCH_SOURCES}
        src/LidarVisualization.cpp
)

add_executable(RnELidar src/LidarDistanceAngleTest.cpp ${SDK_SOURCES})

target_link_libraries(RnELidar pthread rt ${OpenCV_LIBS})