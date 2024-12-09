cmake_minimum_required(VERSION 2.8.3)
project(trac_ik_solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED
        COMPONENTS
        trac_ik_lib
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES trac_solver
        CATKIN_DEPENDS
        trac_ik_lib
        DEPENDS
        Boost
        orocos_kdl
        pkg_nlopt
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${orocos_kdl_INCLUDE_DIRS}
        ${pkg_nlopt_INCLUDE_DIRS}
)

add_executable(ik_tests test/ik_tests.cpp)
target_link_libraries(ik_tests
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${orocos_kdl_LIBRARIES}
        ${pkg_nlopt_LIBRARIES}
)

add_executable(tests_trac_solver test/test_trac_solver.cpp)
target_link_libraries(tests_trac_solver
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${orocos_kdl_LIBRARIES}
        ${pkg_nlopt_LIBRARIES}
)

add_library(trac_solver
        src/tracSolver.cpp
)

target_link_libraries(trac_solver
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${orocos_kdl_LIBRARIES}
        ${pkg_nlopt_LIBRARIES}
)

install(TARGETS trac_solver
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

