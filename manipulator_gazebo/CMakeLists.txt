cmake_minimum_required(VERSION 3.10)
project(manipulator_gazebo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp

        manipulator_common
        gazebo_dev
        gazebo_ros
        gazebo_ros_control
)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        manipulator_hw_sim
        CATKIN_DEPENDS
        roscpp
        manipulator_common
        gazebo_ros_control
        DEPENDS
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(manipulator_hw_sim
        src/ManipulatorHWSim.cpp
)

add_dependencies(manipulator_hw_sim
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(manipulator_hw_sim
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target manipulator_hw_sim")
    add_clang_tooling(
            TARGETS manipulator_hw_sim
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS manipulator_hw_sim
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

install(FILES manipulator_hw_sim_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)