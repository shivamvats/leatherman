cmake_minimum_required(VERSION 2.8.3)
project(leatherman)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Boost REQUIRED)

find_package(Bullet REQUIRED)

find_package(
    catkin REQUIRED
    COMPONENTS
        angles
        cmake_modules
        geometric_shapes
        geometry_msgs
        moveit_msgs
        octomap_ros
        octomap_msgs
        roscpp
        sensor_msgs
        shape_msgs
        tf
        trajectory_msgs
        urdf
        visualization_msgs)

find_package(orocos_kdl REQUIRED)

find_package(Eigen REQUIRED) # dependent on finding cmake_modules through catkin

find_package(PkgConfig REQUIRED)
pkg_check_modules(log4cxx REQUIRED liblog4cxx)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        angles
        geometric_shapes
        geometry_msgs
        moveit_msgs
        octomap_ros
        octomap_msgs
        sensor_msgs
        shape_msgs
        tf
        trajectory_msgs
        urdf
        visualization_msgs
    DEPENDS Boost BULLET Eigen orocos_kdl)

include_directories(SYSTEM ${BULLET_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS})
include_directories(SYSTEM ${log4cxx_INCLUDE_DIRS})
include_directories(include)

add_library(
    ${PROJECT_NAME}
    src/binvox.cpp
    src/bresenham.cpp
    src/file.cpp
    src/mesh_processing.cpp
    src/mesh_resource.cpp
    src/print.cpp
    src/profile.cpp
    src/stl.cpp
    src/utils.cpp
    src/viz.cpp)
target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${BULLET_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${log4cxx_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
