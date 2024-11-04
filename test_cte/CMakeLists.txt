cmake_minimum_required(VERSION 3.0.2)
project(test_cte)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    std_msgs
    geometry_msgs
    ackermann_msgs
    gazebo_msgs
    tf
    rostest
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

# Install the launch and test files
install(DIRECTORY launch tests
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.launch" PATTERN "*.py" PATTERN "*.csv")

# Add rostest
if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(launch/test_cte.launch)
endif()

