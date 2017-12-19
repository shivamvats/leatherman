/*
 * For reading in Binvox files. (http://www.cs.princeton.edu/~min/binvox/)
 * Binvox is a tool that reads in a mesh file and outputs a voxelized
 * version of it.
 *  * Ubuntu 12.04 and up (GLIBC 2.14):
 *    http://www.cs.princeton.edu/~min/binvox/linux64/binvox?rnd=137424032048747
 * Ubuntu 11.10 and earlier (GLIBC 2.13):
 *    http://www.cs.princeton.edu/~min/binvox/linux64b/binvox
*/

#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <octomap/OcTree.h>
#include <octomap/octomap_types.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/Octomap.h>
#include <moveit_msgs/CollisionObject.h>
#include <octomap_msgs/conversions.h>
#include <geometric_shapes/shape_operations.h>

typedef unsigned char byte;

namespace leatherman {

/* Copied from Wizard Andrew Dornbush */
struct VoxelGrid
{
    int version;
    int depth, height, width;
    int size;
    byte* voxels;
    float tx, ty, tz;
    float scale;

    VoxelGrid() :
        version(0),
        depth(0),
        height(0),
        width(0),
        size(0),
        voxels(0),
        tx(0.0f),
        ty(0.0f),
        tz(0.0f),
        scale(0.0f)
    {
    }
};

bool ReadBinvox(const std::string& filename);

// Doesn't seem to scale the points correctly. Don't use this.
bool InsertMesh(
    const std::string& filename,
    const boost::shared_ptr<octomap::Pointcloud>& point_cloud);

bool convertBinvoxToOctomapMsg(
    const std::string& filename,
    double resolution,
    octomap_msgs::Octomap& msg);

bool convertBinvoxToPointCloudMsg(
    const std::string& filename,
    sensor_msgs::PointCloud& pc);

bool convertBinvoxToVector3d(
    const std::string& filename,
    std::vector<Eigen::Vector3d>& voxels);

/* System call to binvox */
bool createBinvoxFile(
    const std::string& mesh_filename,
    std::string& binvox_filename);

/* System call to binvox2bt (installed with the octomap package) */
bool convertBinvoxToBtSystem(
    const std::string& binvox_filename,
    std::string& bt_filename);

/* Copied from octomap/binvox2bt.cpp */
bool convertBinvoxToBt(
    const std::string& binvox_filename,
    std::string& bt_filename);

bool voxelizeMesh(
    const std::string& filename,
    double resolution,
    std::vector<Eigen::Vector3d>& voxels);

void convertOcTreeToCollisionMap(
    const octomap::OcTree& octree,
    moveit_msgs::CollisionObject& cmap);

void getOccupiedVoxelsInOcTree(
    octomap::OcTree* octree,
    std::vector<Eigen::Vector3d>& voxels);

void getOccupiedVoxelsInCollisionMap(
    const moveit_msgs::CollisionObject& map,
    std::vector<Eigen::Vector3d>& voxels);

/* Uses code from octomap/binvox2bt.cpp */
bool getOccupiedVoxelsInBinvoxFile(
    const std::string& binvox_filename,
    std::vector<Eigen::Vector3d>& voxels);

} // namespace leatherman
