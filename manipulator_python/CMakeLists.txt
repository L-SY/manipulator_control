cmake_minimum_required(VERSION 2.8.3)
project(manipulator_python)

find_package(catkin REQUIRED COMPONENTS
        rospy
        actionlib
        control_msgs
        trajectory_msgs
)

catkin_python_setup()

catkin_package()


catkin_install_python(
        PROGRAMS
        scripts/test_send_trajectory.py
        scripts/test_recorder_trajectory.py
        DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src/manipulator_python/
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/manipulator_python
        FILES_MATCHING PATTERN "*.py"
)
