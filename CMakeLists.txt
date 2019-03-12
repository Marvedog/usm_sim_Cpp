cmake_minimum_required(VERSION 2.8.3)
project(usm_sim)
add_definitions(-std=c++11)

set(Eigen3_DIR /usr/lib/cmake/eigen3)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  usm_msgs
  usm_utils
  geometry_msgs
  Eigen3
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
  /usr/include/eigen3
)

add_executable(${PROJECT_NAME} src/simulation_node.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
