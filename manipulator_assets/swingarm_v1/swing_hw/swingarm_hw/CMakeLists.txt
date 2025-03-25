cmake_minimum_required(VERSION 3.0.2)
project(swingarm_hw)

set(CATKIN_PACKAGE_DEPENDENCIES
        roscpp
        urdf
        realtime_tools
        hardware_interface
        controller_interface
        transmission_interface
        joint_limits_interface
        controller_manager

        hw_msgs
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
add_library(swingarm_hw_library SHARED
            src/control_loop.cpp
            src/hardware_interface.cpp
            src/can_interface/can_bus.cpp
            src/can_interface/socketcan.cpp
            src/can_devices/can_dm_actuator.cpp
            src/can_devices/can_st_imu.cpp
            src/can_devices/can_manager.cpp)

add_dependencies(swingarm_hw_library
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(swingarm_hw_library
        ${catkin_LIBRARIES}
)

# main library
add_executable(${PROJECT_NAME}
        src/control_loop.cpp
        src/hardware_interface.cpp
        src/swingarm_hw.cpp
        src/can_interface/can_bus.cpp
        src/can_interface/socketcan.cpp
        src/can_devices/can_dm_actuator.cpp
        src/can_devices/can_st_imu.cpp
        src/can_devices/can_manager.cpp
)
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)


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

