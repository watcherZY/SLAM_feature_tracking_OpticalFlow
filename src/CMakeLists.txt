cmake_minimum_required(VERSION 2.8.3)
project(vi_slam)
######################
### Cmake flags
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    # msg
    sensor_msgs
    # cv
    cv_bridge
    camera_model
    )

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS} 
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_feature_tracker
    feature_tracker/src/feature_tracker_node.cpp
    feature_tracker/src/parameters.cpp
    feature_tracker/src/feature_tracker.cpp
    ) 

target_link_libraries(${PROJECT_NAME}_feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})
