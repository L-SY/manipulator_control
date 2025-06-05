cmake_minimum_required(VERSION 3.0.2)
project(unified_hw)

set(CATKIN_PACKAGE_DEPENDENCIES
        roscpp
        urdf
        realtime_tools
        hardware_interface
        controller_interface
        transmission_interface
        joint_limits_interface
        controller_manager
        imu_complementary_filter
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
file(GLOB CAN_DEVICES_FILES "src/can_manager/can_devices/*.cpp")
file(GLOB CAN_INTERFACE_FILES "src/can_manager/can_interface/*.cpp")

# main library
add_library(unified_hw_library SHARED
            src/control_loop.cpp
            src/hardware_interface.cpp
            src/can_manager/can_manager.cpp
            ${COMMON_FILES}
            ${CAN_DEVICES_FILES}
            ${CAN_INTERFACE_FILES}
)

add_dependencies(unified_hw_library
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(unified_hw_library
        ${catkin_LIBRARIES}
)

# main library
add_executable(${PROJECT_NAME}
        src/control_loop.cpp
        src/hardware_interface.cpp
        src/can_manager/can_manager.cpp
        src/unified_hw.cpp
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

