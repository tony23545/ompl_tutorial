cmake_minimum_required(VERSION 2.8.3)
project(ompl_tutorial)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(OMPL)

## System dependencies are found with CMake's conventions
find_package(Boost COMPONENTS system program_options filesystem REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ompl_tutorial
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

link_directories(
  ${OMPL_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIR}
)
if(NOT Boost_FOUND)
message(FATAL_ERROR "boost not found")
endif()

add_executable(OptimalPlanning src/OptimalPlanning.cpp)
target_link_libraries(OptimalPlanning ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES})
install(TARGETS OptimalPlanning DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})