SET(PROJECT_NAME ootp_simulator)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
#SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

# Set build flags. Set IS_ARM on odroid board as environment variable
SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")


# Add catkin and required ROS packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  image_transport
  cv_bridge
  sensor_msgs
  pcl_conversions
  pcl_ros
  ootp
  ootp_ros
  mav_msgs
  trajectory_msgs
  nav_msgs
)

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)

# Describe catkin Project
catkin_package(
  DEPENDS Eigen OpenCV
  CATKIN_DEPENDS roscpp image_transport cv_bridge sensor_msgs pcl_conversions pcl_ros 
  ootp mav_msgs trajectory_msgs ootp_ros nav_msgs
  INCLUDE_DIRS include
  LIBRARIES ootp_simulator
)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
# Set link libraries
LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
ADD_LIBRARY(ootp_simulator SHARED
src/simulation_tools.cpp
)
TARGET_LINK_LIBRARIES(ootp_simulator ${LINK_LIBS})

ADD_EXECUTABLE(ootp_gazebo
src/gazebo_main.cpp)
target_link_libraries(ootp_gazebo ${catkin_LIBRARIES} ootp_simulator)
                                    
