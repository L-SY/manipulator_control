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

        swing_hw_msgs
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

file(GLOB COMMON_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/common/*.cpp")
file(GLOB CAN_DEVICES_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/can_devices/*.cpp")
file(GLOB CAN_INTERFACE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/can_interface/*.cpp")

# main library
add_library(swingarm_hw_library SHARED
            src/control_loop.cpp
            src/hardware_interface.cpp
            ${COMMON_FILES}
            ${CAN_DEVICES_FILES}
            ${CAN_INTERFACE_FILES}
)

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
        ${COMMON_FILES}
        ${CAN_DEVICES_FILES}
        ${CAN_INTERFACE_FILES}
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

