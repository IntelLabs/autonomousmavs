cmake_minimum_required(VERSION 2.8.3)
project(path_contour)

set(CMAKE_CXX_FLAGS "-std=c++11 -O3 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include
)

catkin_package(
 INCLUDE_DIRS include 
 CATKIN_DEPENDS  std_msgs geometry_msgs         
 LIBRARIES path_contour               
)        

ADD_LIBRARY(path_contour SHARED
src/path_contour/path_contour.cpp
)

target_link_libraries(path_contour
  ${catkin_LIBRARIES}
)




