cmake_minimum_required(VERSION 3.10)
project(gripper_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)

set(CATKIN_PACKAGE_DEPENDENCIES
        roscpp

        effort_controllers
        pluginlib
        controller_interface
        realtime_tools
        manipulator_msgs
        urdf
        manipulator_common
        dynamic_reconfigure
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/GripperController.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIR}
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        LIBRARIES
        ${PROJECT_NAME}
        DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}

)

## Declare cpp executables
add_library(${PROJECT_NAME}
        src/gripper_controller.cpp
)

add_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

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
        FILES gripper_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark other files for installation
#install(
#        DIRECTORY test
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############

#if (${CATKIN_ENABLE_TESTING})
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#  ## Add gtest based cpp test target and link libraries
#  catkin_add_gtest(${PROJECT_NAME}-test
#      test/test_ros_package_template.cpp
#      test/AlgorithmTest.cpp)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#endif ()
