cmake_minimum_required(VERSION 3.0.2)
project(button_position_controller)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
        controller_interface
        hardware_interface
        pluginlib
        roscpp
        std_msgs
        realtime_tools
        swingarm_hw
)

# Declare catkin package
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        controller_interface
        hardware_interface
        pluginlib
        roscpp
        std_msgs
        realtime_tools
        swingarm_hw
)

# Include directories
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# Build the controller library
add_library(${PROJECT_NAME}
        src/button_position_controller.cpp
)

# Link against catkin libraries
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install plugin description
install(FILES button_position_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
