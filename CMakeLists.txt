cmake_minimum_required(VERSION 3.1)
project(scbot)
set(CMAKE_CXX_STANDARD 17) 

# Options
option(BUILD_TESTS "Build test programs" ON)

# telling to use Clang's ld (istead of gcc), in /usr/bin/ld.lld-10
# check https://releases.llvm.org/14.0.0/tools/lld/docs/index.html
# https://cmake.org/cmake/help/latest/envvar/LDFLAGS.html
# otherwise error occurs, 
# clang: error: unable to execute command: Executable "ld" doesn't exist!
set(CMAKE_EXE_LINKER_FLAGS "-fuse-ld=lld")
set(CMAKE_MODULE_LINKER_FLAGS "-fuse-ld=lld")
set(CMAKE_SHARED_LINKER_FLAGS "-fuse-ld=lld")
# set(CMAKE_STATIC_LINKER_FLAGS "-fuse-ld=lld")

if(NOT CMAKE_BUILD_TYPE )
  message("setting CMAKE_BUILD_TYPE: DEBUG")
  set( CMAKE_BUILD_TYPE Debug)
else()
  message("CMAKE_BUILD_TYPE: " ${CMAKE_BUILD_TYPE})
endif ()

set(CMAKE_CXX_FLAGS "-Wall -Wextra ")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -fno-limit-debug-info")

## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   sensor_msgs
#   std_msgs
# )
find_package(Eigen3 REQUIRED)
find_package(rs_driver REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
# message("PCL_DEFINITIONS:" ${PCL_DEFINITIONS})
# include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

## catkin specific configuration ##
# catkin_package(
#   INCLUDE_DIRS include
# )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  # ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${rs_driver_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   map/${PROJECT_NAME}/patrol_robot.cpp
# )
# target_link_libraries(your_project ${rs_driver_LIBRARIES} ${catkin_LIBRARIES})

add_library(pose_transformer src/pose_transformer.cpp)
target_link_libraries(pose_transformer)

# add_executable(test_main src/test_main.cpp)
# target_link_libraries(test_main pose_transformer)

add_library(solar_panel_param_estimator src/solar_panel_param_estimator.cpp)
target_link_libraries(solar_panel_param_estimator pose_transformer ${PCL_LIBRARIES})

add_library(simple_obstacle_detector src/simple_obstacle_detector.cpp)
target_link_libraries(simple_obstacle_detector pose_transformer ${PCL_LIBRARIES})

add_executable(handle_pcap src/handle_pcap.cpp)
target_link_libraries(handle_pcap solar_panel_param_estimator simple_obstacle_detector ${rs_driver_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcap_to_bin src/pcap_to_bin.cpp)
target_link_libraries(pcap_to_bin pose_transformer ${rs_driver_LIBRARIES} ${PCL_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_patrol_robot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# Plain GTest
if(BUILD_TESTS)
  message("Build GTests ON")
  add_subdirectory(tests)
endif()
