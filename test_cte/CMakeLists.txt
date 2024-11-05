cmake_minimum_required(VERSION 3.0.2)
project(test_cross_track_error)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  rostest
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})

## Add rostest
if(CATKIN_ENABLE_TESTING)
  add_rostest(test/pure_pursuit_cte_test.test)
endif()
