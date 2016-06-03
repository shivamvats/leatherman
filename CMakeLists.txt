cmake_minimum_required(VERSION 2.8.3)
project(leatherman)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
             COMPONENTS
                angles 
                geometric_shapes 
                geometry_msgs 
                arm_navigation_msgs
                octomap_ros
                octomap_msgs
                roscpp 
                sensor_msgs 
                shape_msgs
                tf 
                trajectory_msgs
                urdf 
                visualization_msgs
		cmake_modules)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS
                   angles 
                   geometric_shapes 
                   geometry_msgs
                   arm_navigation_msgs 
                   octomap_ros
                   octomap_msgs 
                   sensor_msgs 
                   shape_msgs
                   tf 
                   trajectory_msgs
                   urdf 
                   visualization_msgs
               DEPENDS boost eigen orocos_kdl
               LIBRARIES ${PROJECT_NAME})

add_definitions(${EIGEN_DEFINITIONS})

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

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
