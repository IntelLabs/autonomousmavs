SET(PROJECT_NAME rlmap)
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
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Add catkin and required ROS packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
)

# Add plain cmake packages 
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
find_package(TBB REQUIRED)

# Describe catkin Project
catkin_package(
  DEPENDS Eigen OpenCV
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
  LIBRARIES rlmap
)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBS}
  ${TBB_LIBS}
  ${catkin_LIBRARIES}
)
ADD_LIBRARY(rlmap SHARED
src/config.cpp
src/voxel.cpp
src/map_storage.cpp
src/rlmap.cpp
src/ootp_map.cpp
src/probability_manager.cpp
src/tictoc.cpp
	
)
TARGET_LINK_LIBRARIES(rlmap ${LINK_LIBS})
