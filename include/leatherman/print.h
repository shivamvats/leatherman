#ifndef _LEATHERMAN_PRINT_
#define _LEATHERMAN_PRINT_

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <ros/console.h>
#include <vector>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>

namespace leatherman {

void printPose(const std::vector<double>& p, const std::string& text);

void printPoseMsg(const geometry_msgs::Pose& p, const std::string& text);

void printPoseStampedMsg(
    const geometry_msgs::PoseStamped& p,
    const std::string& text);

void printJointTrajectory(
    const trajectory_msgs::JointTrajectory& traj,
    const std::string& text);

void printJointTrajectoryPoints(
    const std::vector<trajectory_msgs::JointTrajectoryPoint>& points,
    const std::string& text);

void printCompleteJointTrajectory(
    const trajectory_msgs::JointTrajectory& traj,
    const std::string& name);

void printAffine3d(const Eigen::Affine3d& a, const std::string& text);

void printKDLFrame(const KDL::Frame& f, const std::string& text);

void printKDLFrames(
    const std::vector<std::vector<KDL::Frame>>& f,
    const std::string& text);

void printKDLChain(const KDL::Chain& c, const std::string& text);

std::string getString(const std::vector<double>& v, int precision = 3);

std::string getString(
    const std::vector<bool>& v,
    const std::string& t,
    const std::string& f);

template <typename T>
std::string vectorToString(const std::vector<T>& v)
{
    std::stringstream ss;
    ss << "[ ";
    for (size_t i = 0; i < v.size(); ++i) {
        ss << v[i];
        if (i != v.size() - 1) {
            ss << ", ";
        }
    }
    ss << "]";
    return ss.str();
}

} // namespace leatherman

namespace std { // WARNING: non-standard

template <class T, class Allocator>
ostream& operator<<(ostream& o, const vector<T, Allocator>& v)
{
    o << "[ ";
    for (size_t i = 0; i < v.size(); ++i) {
        o << v[i];
        if (i != v.size() - 1) {
            o << ", ";
        }
    }
    o << " ]";
    return o;
}

template <class T, class Allocator>
string to_string(const vector<T, Allocator>& v)
{
    stringstream ss;
    ss << v;
    return ss.str();
}

} // namespace std

#endif
