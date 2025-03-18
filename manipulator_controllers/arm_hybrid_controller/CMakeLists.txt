cmake_minimum_required(VERSION 3.10)
project(arm_hybrid_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp

        arm_common
        arm_msgs
        control_toolbox
        effort_controllers
        pluginlib
        controller_interface
        realtime_tools
        )
find_package(pinocchio REQUIRED)
###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp

        arm_common
        arm_msgs
        control_toolbox
        effort_controllers
        pluginlib
        controller_interface
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare cpp executables
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}.cpp
        include/dynamics_interface/dynamics_interface.h
        include/arm_hybrid_controller/controller_state_interface.h
        include/arm_hybrid_controller/joints_interface.h

        include/trajectory_interface/pos_vel_acc_state.h
        include/trajectory_interface/quintic_spline_segment.h
        include/trajectory_interface/trajectory_interface.h

        include/joint_trajectory_interface/joint_trajectory_segment.h
        include/joint_trajectory_interface/tolerances.h
        include/joint_trajectory_interface/joint_trajectory_msg_utils.h
        include/joint_trajectory_interface/init_joint_trajectory.h
        include/joint_trajectory_interface/creat_joint_trajectory.h
        include/joint_trajectory_interface/trajectory_builder.h
        include/joint_trajectory_interface/hold_trajectory_builder.h
        include/joint_trajectory_interface/stop_trajectory_builder.h
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)
install(
        DIRECTORY include/dynamics_interface/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)
install(
        DIRECTORY include/trajectory_interface/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)
install(
        FILES arm_hybrid_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark other files for installation
install(
        DIRECTORY test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

add_executable(test_pinocchio
        test/test_pinocchio.cpp)
target_link_libraries(test_pinocchio pinocchio::pinocchio)

add_executable(test_quintic_spline_segment
        test/test_quintic_spline_segment.cpp)
target_link_libraries(test_quintic_spline_segment ${catkin_LIBRARIES})
#if (${CATKIN_ENABLE_TESTING})
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#  ## Add gtest based cpp test target and link libraries
#  catkin_add_gtest(${PROJECT_NAME}-test
#      test/test_ros_package_template.cpp
#      test/AlgorithmTest.cpp)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#endif ()
