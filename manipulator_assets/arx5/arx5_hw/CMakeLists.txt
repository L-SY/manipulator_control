cmake_minimum_required(VERSION 3.10)
project(arx5_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        controller_manager
        urdf
)

find_package(kdl_parser REQUIRED)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        controller_manager
        urdf
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/arx_lib/arm_control.cpp
        src/ARX5HW.cpp
        src/ARX5HWLoop.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${kdl_parser_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/arx_lib/libarxx86.so
)


add_executable(arx5_hw_run
        src/arx5_hw.cpp
        src/ARX5HW.cpp
        src/ARX5HWLoop.cpp
)

target_link_libraries(arx5_hw_run
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
        ${kdl_parser_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/arx_lib/libarxx86.so
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)
