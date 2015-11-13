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

inline std::string pad(const char* fmt, ...)
{
    // parse arguments and pass resultant string to configured logger
    va_list args;
    va_start(args, fmt);

    const int buffer_size = 120 + 1;
    char buffer[buffer_size];
    memset(buffer, ' ', buffer_size);
    buffer[buffer_size - 1] = '\0';

    int retval = vsnprintf(buffer, buffer_size, fmt, args);

    if (retval < 0) {
        fprintf(stderr, "sbpl_printall::error, could not complete call to vsnprintf()\n");
    }
    else {
        if (retval < buffer_size - 1) {
            buffer[retval] = ' ';
        }
    }

    va_end(args);
  return std::string(buffer);
}

} // namespace leatherman

#define ROS_INFO_PRETTY(fmt, ...)   ROS_INFO("%s", leatherman::pad(fmt, ##__VA_ARGS__).c_str());
#define ROS_WARN_PRETTY(fmt, ...)   ROS_WARN("%s", leatherman::pad(fmt, ##__VA_ARGS__).c_str());
#define ROS_ERROR_PRETTY(fmt, ...)  ROS_ERROR("%s", leatherman::pad(fmt, ##__VA_ARGS__).c_str());
#define ROS_INFO_PRETTY_NAMED(stream, fmt, ...)   ROS_INFO_NAMED(stream, "%s", leatherman::pad(fmt, ##__VA_ARGS__).c_str());
#define ROS_WARN_PRETTY_NAMED(stream, fmt, ...)   ROS_WARN_NAMED(stream, "%s", leatherman::pad(fmt, ##__VA_ARGS__).c_str());
#define ROS_ERROR_PRETTY_NAMED(stream, fmt, ...)  ROS_ERROR_NAMED(stream, "%s", leatherman::pad(fmt, ##__VA_ARGS__).c_str());

namespace leatherman
{

void printPose(const std::vector<double> &p, std::string text);
void printPoseMsg(const geometry_msgs::Pose &p, std::string text);
void printPoseStampedMsg(const geometry_msgs::PoseStamped &p, std::string text);
void printJointTrajectory(
    const trajectory_msgs::JointTrajectory &traj,
    std::string text);
void printJointTrajectoryPoints(
    const std::vector<trajectory_msgs::JointTrajectoryPoint> &points,
    std::string text);
void printCompleteJointTrajectory(
    const trajectory_msgs::JointTrajectory &traj,
    std::string name);

void printAffine3d(const Eigen::Affine3d &a, std::string text);
void printKDLFrame(const KDL::Frame &f, std::string text);
void printKDLFrames(
    const std::vector<std::vector<KDL::Frame>>& f,
    std::string text);
void printKDLChain(const KDL::Chain &c, std::string text);

std::string getString(const std::vector<double> &v, int precision=3);
std::string getString(const std::vector<bool> &v, std::string t, std::string f);

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
string to_string(const std::vector<T, Allocator>& v)
{
    stringstream ss;
    ss << v;
    return ss.str();
}

} // namespace std

#endif
