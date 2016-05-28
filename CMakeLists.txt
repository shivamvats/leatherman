cmake_minimum_required(VERSION 2.8.3)
project(leatherman)

set(CMAKE_BUILD_TYPE Release)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(catkin REQUIRED
             COMPONENTS
                angles 
                geometric_shapes 
                geometry_msgs 
                moveit_msgs
                octomap_ros
                octomap_msgs
                orocos_kdl 
                roscpp 
                sensor_msgs 
                shape_msgs
                tf 
                trajectory_msgs
                urdf 
                visualization_msgs)
include_directories(${catkin_INCLUDE_DIRS})

include_directories(include)

catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS
                   angles 
                   geometric_shapes 
                   geometry_msgs
                   moveit_msgs 
                   octomap_ros
                   octomap_msgs 
                   orocos_kdl 
                   sensor_msgs 
                   shape_msgs
                   tf 
                   trajectory_msgs
                   urdf 
                   visualization_msgs
               DEPENDS boost eigen
               LIBRARIES ${PROJECT_NAME})

add_library(${PROJECT_NAME}
            src/binvox.cpp
            src/bresenham.cpp
            src/file.cpp
            src/print.cpp
            src/stl.cpp
            src/utils.cpp 
            src/viz.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
