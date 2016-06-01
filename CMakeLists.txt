# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(leatherman)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp tf angles sensor_msgs geometry_msgs geometric_shapes trajectory_msgs visualization_msgs arm_navigation_msgs shape_msgs orocos_kdl urdf eigen)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp  tf angles sensor_msgs geometry_msgs geometric_shapes trajectory_msgs visualization_msgs arm_navigation_msgs shape_msgs orocos_kdl urdf eigen
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}# TODO include
    LIBRARIES ${catkin_LIBRARIES}# TODO
)
# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)
add_definitions(${EIGEN_DEFINITIONS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME}  src/utils.cpp 
                                      src/print.cpp
                                      src/file.cpp
                                      src/viz.cpp
                                      src/bresenham.cpp
                                      src/objects.cpp
                                      src/binvox.cpp
                                      src/stl.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
