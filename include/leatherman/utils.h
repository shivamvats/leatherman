/* \author Benjamin Cohen */

#ifndef _LEATHERMAN_UTILS_
#define _LEATHERMAN_UTILS_

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometric_shapes/mesh_operations.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/MultiDOFJointState.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/LinearMath/Scalar.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <urdf/model.h>

namespace leatherman {

////////////////////////
// Geometry Utilities //
////////////////////////

double distanceBetween3DLineSegments(
    const Eigen::Vector3d& l1a, const Eigen::Vector3d& l1b,
    const Eigen::Vector3d& l2a, const Eigen::Vector3d& l2b);

///////////////////////////
// Joint State Utilities //
///////////////////////////

bool isValidJointState(const sensor_msgs::JointState& state);
bool isValidMultiDOFJointState(const sensor_msgs::MultiDOFJointState& state);

bool findJointPosition(
    const sensor_msgs::JointState& state,
    std::string name,
    double& position);

/// \brief Extract a subset of joints from a joint state message.
bool getJointPositions(
    const sensor_msgs::JointState& joint_state,
    const sensor_msgs::MultiDOFJointState& multi_dof_joint_state,
    const std::vector<std::string>& joint_names,
    std::vector<double>& positions);

/// \brief Extract a subset of joints from a joint state message and report any
///     missing joints.
bool getJointPositions(
    const sensor_msgs::JointState& joint_state,
    const sensor_msgs::MultiDOFJointState& multi_dof_joint_state,
    const std::vector<std::string>& joint_names,
    std::vector<double>& positions,
    std::vector<std::string>& missing);

void findAndReplaceJointPosition(
    std::string name,
    double position,
    sensor_msgs::JointState& state);

///////////////////
// KDL Utilities //
///////////////////

bool getJointIndex(const KDL::Chain& c, std::string name, int& index);

bool getSegmentIndex(const KDL::Chain& c, std::string name, int& index);

bool getSegmentOfJoint(
    const KDL::Tree& tree,
    std::string joint,
    std::string& segment);

bool getChainTip(
    const KDL::Tree& tree,
    const std::vector<std::string>& segments,
    std::string chain_root, std::string& chain_tip);

////////////
// Colors //
////////////

/// \brief Convert an (H, S, V) color triplet to (R, G, B) format.
/// \param h The hue in range [0, 360)
/// \param s The saturation in range [0, 1]
/// \param v The value in range [0, 1]
void HSVtoRGB(double* r, double* g, double* b, double h, double s, double v);
void msgRGBToHSV(const std_msgs::ColorRGBA& color, double& h, double& s, double& v);
void msgHSVToRGB(double h, double s, double v, std_msgs::ColorRGBA& color);

/////////////////
// ROS Logging //
/////////////////

void setLoggerLevel(std::string package, std::string name, std::string level);
void setLoggerLevel(std::string name, std::string level);

} // namespace leatherman

#endif
