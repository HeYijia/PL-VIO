cmake_minimum_required(VERSION 2.8.3)
project(pub_imu_features)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    )

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(pub_imu src/pub_imu.cpp)
target_link_libraries(pub_imu ${catkin_LIBRARIES})

add_executable(pub_feature src/pub_feature.cpp)
target_link_libraries(pub_feature ${catkin_LIBRARIES})

add_executable(pub_stereo_feature src/pub_stereo_feature.cpp)
target_link_libraries(pub_stereo_feature ${catkin_LIBRARIES})


