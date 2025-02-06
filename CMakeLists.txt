cmake_minimum_required(VERSION 3.1)
project(scbot)
set(CMAKE_CXX_STANDARD 17) 

# Options
option(BUILD_TESTS "Build test programs" ON)

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

add_executable(handle_pcap src/handle_pcap.cpp)
target_link_libraries(handle_pcap solar_panel_param_estimator ${rs_driver_LIBRARIES} ${PCL_LIBRARIES})

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
