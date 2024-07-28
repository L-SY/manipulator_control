cmake_minimum_required(VERSION 3.0.2)
project(robot_hw)

set(CATKIN_PACKAGE_DEPENDENCIES
        roscpp
        urdf
        realtime_tools
        hardware_interface
        controller_interface
        transmission_interface
        joint_limits_interface
        controller_manager

        robot_msgs
        robot_common
)

find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        LIBRARIES
        ${PROJECT_NAME}
        DEPENDS
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# main library
file(GLOB_RECURSE MAIN_LIBRARY_CPP
        "src/*.cpp")
add_library(${PROJECT_NAME} SHARED ${MAIN_LIBRARY_CPP})

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

# executable cpp
#add_executable(xxxx_name
#        src/xxx.cpp
#)
#add_dependencies(xxxx_name
#        ${PROJECT_NAME}
#        ${catkin_EXPORTED_TARGETS}
#)
#target_link_libraries(xxxx_name
#        ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

