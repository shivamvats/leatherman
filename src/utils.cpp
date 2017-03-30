#include <leatherman/utils.h>
#include <leatherman/print.h>
//#include <geometric_shapes/mesh_operations.h>
#include <resource_retriever/retriever.h>
#include <tinyxml.h>
#include <urdf/model.h>
#include <log4cxx/logger.h>

#define SMALL_NUM  0.00000001     // to avoid division overflow

double leatherman::distanceBetween3DLineSegments(
    const Eigen::Vector3d& l1a,
    const Eigen::Vector3d& l1b,
    const Eigen::Vector3d& l2a,
    const Eigen::Vector3d& l2b)
{
    // Copyright 2001, softSurfer (www.softsurfer.com)
    // This code may be freely used and modified for any purpose
    // providing that this copyright notice is included with it.
    // SoftSurfer makes no warranty for this code, and cannot be held
    // liable for any real or imagined damage resulting from its use.
    // Users of this code must verify correctness for their application.

    double u[3];
    double v[3];
    double w[3];
    double dP[3];

    u[0] = l1b[0] - l1a[0];
    u[1] = l1b[1] - l1a[1];
    u[2] = l1b[2] - l1a[2];

    v[0] = l2b[0] - l2a[0];
    v[1] = l2b[1] - l2a[1];
    v[2] = l2b[2] - l2a[2];

    w[0] = l1a[0] - l2a[0];
    w[1] = l1a[1] - l2a[1];
    w[2] = l1a[2] - l2a[2];

    double a = u[0] * u[0] + u[1] * u[1] + u[2] * u[2]; // always >= 0
    double b = u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; // dot(u,v);
    double c = v[0] * v[0] + v[1] * v[1] + v[2] * v[2]; // dot(v,v);        // always >= 0
    double d = u[0] * w[0] + u[1] * w[1] + u[2] * w[2]; // dot(u,w);
    double e = v[0] * w[0] + v[1] * w[1] + v[2] * w[2]; // dot(v,w);
    double D = a*c - b*b;       // always >= 0
    double sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
    double tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < SMALL_NUM) { // the lines are almost parallel
        sN = 0.0;        // force using point P0 on segment S1
        sD = 1.0;        // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
             sN = sD;
        else {
            sN = (-d + b);
            sD = a;
        }
    }

    // finally do the division to get sc and tc
    sc = (fabs(sN) < SMALL_NUM ? 0.0 : sN / sD);
    tc = (fabs(tN) < SMALL_NUM ? 0.0 : tN / tD);

    // get the difference of the two closest points
    // dP = w + (sc * u) - (tc * v);  // = S1(sc) - S2(tc)

    dP[0] = w[0] + (sc * u[0]) - (tc * v[0]);
    dP[1] = w[1] + (sc * u[1]) - (tc * v[1]);
    dP[2] = w[2] + (sc * u[2]) - (tc * v[2]);

    return  sqrt(dP[0]*dP[0] + dP[1]*dP[1] + dP[2]*dP[2]);   // return the closest distance
}

/* Originally from geometric_shapes/shape_operations. Then it
 * was moved to pr2_navigation/load_mesh.cpp
 * Written by Ioan Sucan */

/*
shapes::Mesh* leatherman::createMeshFromBinaryStlData(const char *data, unsigned int size)
{
  const char* pos = data;
  pos += 80; // skip the 80 byte header

  unsigned int numTriangles = *(unsigned int*)pos;
  pos += 4;

  // make sure we have read enough data
  if ((long)(50 * numTriangles + 84) <= size)
  {
    std::vector<tf::Vector3> vertices;

    for (unsigned int currentTriangle = 0 ; currentTriangle < numTriangles ; ++currentTriangle)
    {
      // skip the normal
      pos += 12;

      // read vertices
      tf::Vector3 v1(0,0,0);
      tf::Vector3 v2(0,0,0);
      tf::Vector3 v3(0,0,0);

      v1.setX(*(float*)pos);
      pos += 4;
      v1.setY(*(float*)pos);
      pos += 4;
      v1.setZ(*(float*)pos);
      pos += 4;

      v2.setX(*(float*)pos);
      pos += 4;
      v2.setY(*(float*)pos);
      pos += 4;
      v2.setZ(*(float*)pos);
      pos += 4;

      v3.setX(*(float*)pos);
      pos += 4;
      v3.setY(*(float*)pos);
      pos += 4;
      v3.setZ(*(float*)pos);
      pos += 4;

      // skip attribute
      pos += 2;

      vertices.push_back(v1);
      vertices.push_back(v2);
      vertices.push_back(v3);
    }

    return shapes::createMeshFromVertices(vertices);
  }

  return NULL;
}

shapes::Mesh* leatherman::createMeshFromBinaryStl(const char *filename)
{
  FILE* input = fopen(filename, "r");
  if (!input)
    return NULL;

  fseek(input, 0, SEEK_END);
  long fileSize = ftell(input);
  fseek(input, 0, SEEK_SET);

  char* buffer = new char[fileSize];
  size_t rd = fread(buffer, fileSize, 1, input);

  fclose(input);

  shapes::Mesh *result = NULL;

  if (rd == 1)
    result = createMeshFromBinaryStlData(buffer, fileSize);

  delete[] buffer;

  return result;
}
*/

void leatherman::getMeshComponents(
    shapes::Mesh* mesh,
    std::vector<int>& triangles,
    std::vector<Eigen::Vector3d>& vertices)
{
    Eigen::Vector3d v;

    // copy vertices
    ROS_DEBUG("vertexCount: %d    triangleCount: %d", mesh->vertex_count, mesh->triangle_count);
    for (unsigned int i = 0; i < mesh->vertex_count; ++i) {
        v.x() = mesh->vertices[3 * i    ];
        v.y() = mesh->vertices[3 * i + 1];
        v.z() = mesh->vertices[3 * i + 2];
        ROS_DEBUG("[vertex %d] xyz: %0.3f %0.3f %0.3f", int(i), v.x(), v.y(), v.z());
        vertices.push_back(v);
    }

    // copy triangles
    triangles.resize(3 * mesh->triangle_count);
    for (unsigned int i = 0; i < 3 * mesh->triangle_count; ++i) {
        triangles[i] = mesh->triangles[i];
    }
}

void leatherman::rpyToQuatMsg(double r, double p, double y, geometry_msgs::Quaternion &q)
{
  tf::Quaternion btpose;
  btpose = setRPY(r, p, y);
  tf::quaternionTFToMsg(btpose, q);
}

void leatherman::getRPY(const geometry_msgs::Quaternion &qmsg, double &roll, double &pitch, double &yaw)
{
  geometry_msgs::Pose pose;
  pose.orientation = qmsg;
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose, tf_pose);
  tf_pose.getBasis().getRPY(roll,pitch,yaw);
  ROS_DEBUG("[utils] rpy: %0.3f %0.3f %0.3f  quat: %0.3f %0.3f %0.3f %0.3f\n", roll, pitch, yaw, qmsg.x, qmsg.y, qmsg.z, qmsg.w);fflush(stdout);
}

bool leatherman::getIntermediatePoints(trajectory_msgs::JointTrajectoryPoint a, trajectory_msgs::JointTrajectoryPoint b, int num_points, std::vector<trajectory_msgs::JointTrajectoryPoint> &points)
{
  if(a.positions.size() != b.positions.size())
    return false;

  double time_inc = (b.time_from_start - a.time_from_start).toSec() / (num_points+1);
  std::vector<double> inc(a.positions.size(),0);
  for(size_t i = 0; i < a.positions.size(); ++i)
    inc[i] = angles::shortest_angular_distance(a.positions[i], b.positions[i]) / (num_points+1);


  points.resize(num_points);
  for(int i = 0; i < num_points; ++i)
  {
    points[i].positions.resize(a.positions.size());
    for(size_t j = 0; j < a.positions.size(); ++j)
      points[i].positions[j] = a.positions[j] + (i+1)*inc[j];
    points[i].time_from_start = a.time_from_start + ros::Duration((i+1)*(time_inc));
  }
  return true;
}

tf::Quaternion leatherman::setRPY(const tfScalar& roll, const tfScalar& pitch, const tfScalar& yaw)
{
  tfScalar halfYaw = tfScalar(yaw) * tfScalar(0.5);
  tfScalar halfPitch = tfScalar(pitch) * tfScalar(0.5);
  tfScalar halfRoll = tfScalar(roll) * tfScalar(0.5);
  tfScalar cosYaw = tfCos(halfYaw);
  tfScalar sinYaw = tfSin(halfYaw);
  tfScalar cosPitch = tfCos(halfPitch);
  tfScalar sinPitch = tfSin(halfPitch);
  tfScalar cosRoll = tfCos(halfRoll);
  tfScalar sinRoll = tfSin(halfRoll);
  tf::Quaternion q;
  q.setValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
      cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
      cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
      cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx

  return q;
}

void leatherman::setRPY(double roll, double pitch, double yaw, Eigen::Matrix3d &m)
{
  m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

void leatherman::getRPY(const Eigen::Matrix3d &m, double &roll, double &pitch, double &yaw)
{
  Eigen::Vector3d v = m.eulerAngles(0,1,2);
  roll = v(0);  pitch = v(1);  yaw = v(2);
}

void leatherman::getIntermediatePoints(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    double d,
    std::vector<Eigen::Vector3d>& points)
{
  Eigen::Vector3d pt, dir;
  int interm_points = floor((a - b).norm() / d + 0.5);

  dir = b - a;
  dir.normalize();
  ROS_DEBUG("# interm points: %d  unit vector: %0.3f %0.3f %0.3f", interm_points, dir(0), dir(1), dir(2));

  points.clear();
  points.push_back(a);
  for(int i = 1; i <= interm_points; ++i)
  {
    pt = a + dir*i*d;
    points.push_back(pt);
  }
  points.push_back(b);
}

bool leatherman::isValidJointState(const sensor_msgs::JointState& state)
{
    return (state.position.empty() || state.position.size() == state.name.size()) ||
            (state.velocity.empty() || state.velocity.size() == state.name.size()) ||
            (state.effort.empty() || state.effort.size() == state.name.size());
}

bool leatherman::isValidMultiDOFJointState(const sensor_msgs::MultiDOFJointState& state)
{
    return (state.transforms.empty() || state.transforms.size() == state.joint_names.size()) ||
            (state.twist.empty() || state.twist.size() == state.joint_names.size()) ||
            (state.wrench.empty() || state.wrench.size() == state.joint_names.size());
}

bool leatherman::findJointPosition(const sensor_msgs::JointState &state, std::string name, double &position)
{
  for(size_t i = 0; i < state.name.size(); i++)
  {
    if(name.compare(state.name[i]) == 0)
    {
      position = state.position[i];
      return true;
    }
  }
  return false;
}

bool leatherman::getJointPositions(
    const sensor_msgs::JointState& joint_state,
    const sensor_msgs::MultiDOFJointState& multi_dof_joint_state,
    const std::vector<std::string>& joint_names,
    std::vector<double>& positions)
{
    std::vector<std::string> missing;
    return getJointPositions(
            joint_state,
            multi_dof_joint_state,
            joint_names,
            positions,
            missing);
}

bool leatherman::getJointPositions(
    const sensor_msgs::JointState& joint_state,
    const sensor_msgs::MultiDOFJointState& multi_dof_joint_state,
    const std::vector<std::string>& joint_names,
    std::vector<double>& positions,
    std::vector<std::string>& missing)
{
    if (!isValidJointState(joint_state) ||
        !isValidMultiDOFJointState(multi_dof_joint_state))
    {
        return false;
    }

    positions.resize(joint_names.size());
    missing.clear();

    for (size_t jidx = 0; jidx < joint_names.size(); ++jidx) {
        const std::string& joint_name = joint_names[jidx];
        const size_t idx = joint_name.find('/');
        if (idx != std::string::npos) {
            std::string jname = joint_name.substr(0, idx);
            std::string local_var_name = joint_name.substr(idx + 1);
            auto it = std::find(
                    multi_dof_joint_state.joint_names.begin(),
                    multi_dof_joint_state.joint_names.end(),
                    jname);
            if (it == multi_dof_joint_state.joint_names.end()) {
                missing.push_back(joint_name);
            } else {
                size_t jind = std::distance(multi_dof_joint_state.joint_names.begin(), it);
                const auto& trans = multi_dof_joint_state.transforms[jind];
                if (local_var_name == "x" || local_var_name == "trans_x") {
                    positions[jidx] = trans.translation.x;
                } else if (local_var_name == "y" || local_var_name == "trans_y") {
                    positions[jidx] = trans.translation.y;
                } else if (local_var_name == "z" || local_var_name == "trans_z") {
                    positions[jidx] = trans.translation.z;
                } else if (local_var_name == "theta") {
                    double theta;

                    Eigen::Quaterniond q(
                            trans.rotation.w,
                            trans.rotation.x,
                            trans.rotation.y,
                            trans.rotation.z);
                    double s_squared = 1.0 - (q.w() * q.w());
                    // from sbpl_collision_checking, from MoveIt, from BULLET
                    if (s_squared < 10.0 * std::numeric_limits<double>::epsilon()) {
                        theta = 0.0;
                    } else {
                        double s = 1.0 / sqrt(s_squared);
                        theta = (acos(q.w()) * 2.0f) * (q.z() * s);
                    }
                    positions[jidx] = theta;
                } else if (local_var_name == "rot_w") {
                    Eigen::Quaterniond q(
                            trans.rotation.w,
                            trans.rotation.x,
                            trans.rotation.y,
                            trans.rotation.z);
                    positions[jidx] = q.w();
                } else if (local_var_name == "rot_x") {
                    Eigen::Quaterniond q(
                            trans.rotation.w,
                            trans.rotation.x,
                            trans.rotation.y,
                            trans.rotation.z);
                    positions[jidx] = q.x();
                } else if (local_var_name == "rot_y") {
                    Eigen::Quaterniond q(
                            trans.rotation.w,
                            trans.rotation.x,
                            trans.rotation.y,
                            trans.rotation.z);
                    positions[jidx] = q.y();
                } else if (local_var_name == "rot_z") {
                    Eigen::Quaterniond q(
                            trans.rotation.w,
                            trans.rotation.x,
                            trans.rotation.y,
                            trans.rotation.z);
                    positions[jidx] = q.z();
                } else {
                    ROS_ERROR("Unrecognized local variable name '%s'", local_var_name.c_str());
                    return false;
                }
            }
        } else {
            auto it = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
            if (it == joint_state.name.end()) {
                missing.push_back(joint_name);
            }
            else {
                size_t jind = std::distance(joint_state.name.begin(), it);
                positions[jidx] = joint_state.position[jind];
            }
        }
    }

    if (!missing.empty()) {
        positions.clear();
        return false;
    }

    return true;
}

void leatherman::findAndReplaceJointPosition(std::string name, double position, sensor_msgs::JointState &state)
{
  bool exists = false;
  for(size_t i = 0; i < state.name.size(); i++)
  {
    if(state.name[i].compare(name) == 0)
    {
      state.position[i] = position;
      exists = true;
    }
  }
  if(!exists)
  {
    state.position.push_back(position);
    state.name.push_back(name);
  }
}

bool leatherman::getJointIndex(const KDL::Chain &c, std::string name, int &index)
{
  for(size_t j = 0; j < c.getNrOfSegments(); ++j)
  {
    if(c.getSegment(j).getJoint().getName().compare(name) == 0)
    {
      index = j;
      return true;
    }
  }
  return false;
}

bool leatherman::getSegmentIndex(const KDL::Chain &c, std::string name, int &index)
{
  for(size_t j = 0; j < c.getNrOfSegments(); ++j)
  {
    if(c.getSegment(j).getName().compare(name) == 0)
    {
      index = j;
      return true;
    }
  }
  return false;
}

bool leatherman::getSegmentOfJoint(const KDL::Tree &tree, std::string joint, std::string &segment)
{
  KDL::SegmentMap smap = tree.getSegments();
  for(std::map<std::string, KDL::TreeElement>::const_iterator iter = smap.begin(); iter != smap.end(); ++iter)
  {
    if(iter->second.segment.getJoint().getName().compare(joint) == 0)
    {
      segment = iter->second.segment.getName();
      return true;
    }
  }
  return false;
}

bool leatherman::getChainTip(const KDL::Tree &tree, const std::vector<std::string> &segments, std::string chain_root, std::string &chain_tip)
{
  KDL::Chain chain;

  // compute # of links each link would include if tip of chain
  for(size_t i = 0; i < segments.size(); ++i)
  {
    // create chain with link i as the tip
    if (!tree.getChain(chain_root, segments[i], chain))
    {
      ROS_ERROR("Failed to fetch the KDL chain. This code only supports a set of segments that live within a single kinematic chain. (root: %s, tip: %s)", chain_root.c_str(), segments[i].c_str());
      return false;
    }

    int index;
    size_t num_segments_included = 0;
    for(size_t j = 0; j < segments.size(); ++j)
    {
      if(leatherman::getSegmentIndex(chain, segments[j], index))
        num_segments_included++;
    }

    if(num_segments_included == segments.size())
    {
      chain_tip = segments[i];
      return true;
    }
  }
  return false;
}

void leatherman::HSVtoRGB(
    double* r, double* g, double* b,
    double h, double s, double v)
{
    int i;
    double f, p, q, t;
    if (s == 0) {
        // achromatic (grey)
        *r = *g = *b = v;
        return;
    }
    h /= 60.0;      // sector 0 to 5
    i = floor(h);
    f = h - i;      // factorial part of h
    p = v * (1.0 - s);
    q = v * (1.0 - s * f);
    t = v * (1.0 - s * (1.0 - f));
    switch (i) {
    case 0:
        *r = v;
        *g = t;
        *b = p;
        break;
    case 1:
        *r = q;
        *g = v;
        *b = p;
        break;
    case 2:
        *r = p;
        *g = v;
        *b = t;
        break;
    case 3:
        *r = p;
        *g = q;
        *b = v;
        break;
    case 4:
        *r = t;
        *g = p;
        *b = v;
        break;
    default:
        *r = v;
        *g = p;
        *b = q;
        break;
    }
}

void leatherman::msgRGBToHSV(
    const std_msgs::ColorRGBA& color,
    double& h, double& s, double& v)
{
    // TODO: implement
}

void leatherman::msgHSVToRGB(
    double h, double s, double v,
    std_msgs::ColorRGBA& color)
{
    double r, g, b;
    leatherman::HSVtoRGB(&r, &g, &b, h, s, v);
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1.0f;
}

void leatherman::setLoggerLevel(std::string package, std::string name, std::string level)
{
  ROSCONSOLE_AUTOINIT;

  //std::string logger_name = ROSCONSOLE_DEFAULT_NAME + std::string(".") + name;
  std::string logger_name = package + std::string(".") + name;

  ROS_INFO("Setting %s to %s level", logger_name.c_str(), level.c_str());

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(logger_name);

  // Set the logger for this package to output all statements
  if(level.compare("debug") == 0)
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  else
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);

  ROS_DEBUG_NAMED(name, "This is a debug statement, and should print if you enabled debug.");
}

void leatherman::setLoggerLevel(std::string name, std::string level)
{
  ROSCONSOLE_AUTOINIT;

  //std::string logger_name = ROSCONSOLE_DEFAULT_NAME + std::string(".") + name;
  std::string logger_name = name;

  ROS_INFO("Setting %s to %s level", logger_name.c_str(), level.c_str());

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(logger_name);

  // Set the logger for this package to output all statements
  if(level.compare("debug") == 0)
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  else
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);

  ROS_DEBUG_NAMED(name, "This is a debug statement, and should print if you enabled debug.");
}

bool leatherman::getPose(
    const sensor_msgs::MultiDOFJointState& state,
    std::string frame_id,
    std::string child_frame_id,
    geometry_msgs::Pose& pose)
{
    ROS_WARN("leatherman::getPose currently unimplemented");
    return false;

//  if(state.frame_ids.size() != state.child_frame_ids.size())
//    return false;
//
//  if(frame_id.compare(child_frame_id) == 0)
//  {
//    pose.position.x = 0;
//    pose.position.y = 0;
//    pose.position.z = 0;
//    pose.orientation.w = 1;
//    return true;
//  }
//
//  for(size_t i = 0; i < state.frame_ids.size(); ++i)
//  {
//    if(state.frame_ids[i].compare(frame_id) == 0)
//    {
//      if(state.child_frame_ids[i].compare(child_frame_id) == 0)
//      {
//        pose = state.poses[i];
//        return true;
//      }
//    }
//  }
//
//  // look for inverse
//  for(size_t i = 0; i < state.frame_ids.size(); ++i)
//  {
//    if(state.child_frame_ids[i].compare(frame_id) == 0)
//    {
//      if(state.frame_ids[i].compare(child_frame_id) == 0)
//      {
//        tf::Transform bt;
//        leatherman::poseMsgTobtTransform(state.poses[i], bt);
//        leatherman::btTransformToPoseMsg(bt.inverse(), pose);
//        return true;
//      }
//    }
//  }
//  return false;
}

bool leatherman::getJointLimits(
    const urdf::Model* urdf,
    const std::string& root_name,
    const std::string& tip_name,
    std::vector<std::string>& joint_names,
    std::vector<double>& min_limits,
    std::vector<double>& max_limits,
    std::vector<bool>& continuous)
{
    // gather joints tracing backwards from the tip link to the root link;
    // bail out if the root link is not encountered
    unsigned int num_joints = 0;
    boost::shared_ptr<const urdf::Link> link = urdf->getLink(tip_name);
    while (link && link->name != root_name) {
        boost::shared_ptr<const urdf::Joint> joint = urdf->getJoint(link->parent_joint->name);
        ROS_DEBUG("adding joint: [%s]", joint->name.c_str() );
        if (!joint) {
            ROS_ERROR("Could not find joint: %s", link->parent_joint->name.c_str());
            return false;
        }
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            num_joints++;
        }
        link = urdf->getLink(link->getParent()->name);
    }
    ROS_DEBUG("%d joints found.", num_joints);

    min_limits.resize(num_joints);
    max_limits.resize(num_joints);
    joint_names.resize(num_joints);
    continuous.resize(num_joints, false);

    // gather limits for all joints between tip and link
    link = urdf->getLink(tip_name);
    unsigned int i = 0;
    while (link && i < num_joints) {
        boost::shared_ptr<const urdf::Joint> joint = urdf->getJoint(link->parent_joint->name);
        ROS_DEBUG("getting bounds for joint: [%s]", joint->name.c_str() );
        if (!joint) {
            ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
            return false;
        }
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            if (joint->type != urdf::Joint::CONTINUOUS) {
                joint_names[num_joints-i-1] = joint->name;
                continuous[num_joints-i-1] = false;

                if (joint->safety == NULL) {
                    min_limits[num_joints-i-1] = joint->limits->lower;
                    max_limits[num_joints-i-1] = joint->limits->upper;
                }
                else {
                    min_limits[num_joints-i-1] = joint->safety->soft_lower_limit;
                    max_limits[num_joints-i-1] = joint->safety->soft_upper_limit;
                }
            }
            else {
                joint_names[num_joints-i-1] = joint->name;
                min_limits[num_joints-i-1] = -M_PI;
                max_limits[num_joints-i-1] = M_PI;
                continuous[num_joints-i-i] = true;
            }
            ROS_INFO("[%s] min: %0.3f  max: %0.3f", joint_names[num_joints-i-1].c_str(), min_limits[num_joints-i-1], max_limits[num_joints-i-1]);
            i++;
        }
        link = urdf->getLink(link->getParent()->name);
    }
    return true;
}

bool leatherman::getJointLimits(
    const urdf::Model* urdf,
    const std::string& root_name,
    const std::string& tip_name,
    const std::string& joint_name,
    double& min_limit,
    double& max_limit,
    bool& continuous)
{
    bool found_joint = false;
    boost::shared_ptr<const urdf::Link> link = urdf->getLink(tip_name);
    while (link && (link->name != root_name) && !found_joint)
    {
        boost::shared_ptr<const urdf::Joint> joint = urdf->getJoint(link->parent_joint->name);
        if (joint->name.compare(joint_name) == 0) {
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
                if (joint->type != urdf::Joint::CONTINUOUS) {
                    continuous = false;
                    if (joint->safety == NULL) {
                        min_limit = joint->limits->lower;
                        max_limit = joint->limits->upper;
                    }
                    else {
                        min_limit = joint->safety->soft_lower_limit;
                        max_limit = joint->safety->soft_upper_limit;
                    }
                }
                else {
                    min_limit = -M_PI;
                    max_limit = M_PI;
                    continuous = true;
                }
            }
            found_joint = true;
        }
        link = urdf->getLink(link->getParent()->name);
    }
    return found_joint;
}

/* Copied from Bullet Physics library */
void leatherman::getRPY(const std::vector<std::vector<double> > &Rot, double* roll, double* pitch, double* yaw, int solution_number)
{
  double delta,rpy1[3],rpy2[3];

  // Check that pitch is not at a singularity
  if(fabs(Rot[0][2]) >= 1)
  {
    rpy1[2]  = 0;
    rpy2[2]  = 0;

    // From difference of angles formula
    delta = atan2(Rot[0][0], Rot[2][0]);
    if(Rot[0][2] > 0)   //gimbal locked up
    {
      rpy1[1] = M_PI / 2.0;
      rpy2[1] = M_PI / 2.0;
      rpy1[0] = rpy1[1] + delta;
      rpy2[0] = rpy2[1] + delta;
    }
    else // gimbal locked down
    {
      rpy1[1] = -M_PI / 2.0;
      rpy2[1] = -M_PI / 2.0;
      rpy1[0] = -rpy1[1] + delta;
      rpy2[0] = -rpy2[1] + delta;
    }
  }
  else
  {
    rpy1[1] = -asin(Rot[0][2]);
    rpy2[1] = M_PI - rpy1[1];


    rpy1[0] = atan2(Rot[1][2]/cos(rpy1[1]),
        Rot[2][2]/cos(rpy1[1]));

    rpy2[0] = atan2(Rot[1][2]/cos(rpy2[1]),
        Rot[2][2]/cos(rpy2[1]));

    rpy1[2] = atan2(Rot[0][1]/cos(rpy1[1]),
        Rot[0][0]/cos(rpy1[1]));

    rpy2[2] = atan2(Rot[0][1]/cos(rpy2[1]),
        Rot[0][0]/cos(rpy2[1]));
  }

  if (solution_number == 1)
  {
    *yaw = rpy1[2];
    *pitch = rpy1[1];
    *roll = rpy1[0];
  }
  else
  {
    *yaw = rpy2[2];
    *pitch = rpy2[1];
    *roll = rpy2[0];
  }
}

bool leatherman::getMeshComponentsFromResource(
    const std::string& resource,
    const Eigen::Vector3d& scale,
    std::vector<int>& triangles,
    std::vector<Eigen::Vector3d>& vertices)
{
    if (resource.empty()) {
        return false;
    }

    shapes::Shape* mesh = shapes::createMeshFromResource(resource, scale);
    if (!mesh) {
        ROS_ERROR("Failed to load mesh '%s'", resource.c_str());
        return false;
    }

    shapes::Mesh* m = static_cast<shapes::Mesh*>(mesh);
    leatherman::getMeshComponents(m, triangles, vertices);
    return true;
}

void leatherman::scaleVertices(const std::vector<Eigen::Vector3d> &vin, double sx, double sy, double sz, std::vector<Eigen::Vector3d> &vout)
{
  // find the mean of the points
  Eigen::Vector3d mean;
  double sumx = 0, sumy = 0, sumz = 0;
  for(size_t i = 0; i < vin.size(); ++i)
  {
    sumx += vin[i].x();
    sumy += vin[i].y();
    sumz += vin[i].z();
  }
  mean(0) = sumx / double(vin.size());
  mean(1) = sumy / double(vin.size());
  mean(2) = sumz / double(vin.size());

  //Eigen::Affine3d scale(Eigen::Translation3d(sx, sy, sz) * Eigen::Affine3d::Identity());
  //Eigen::Vector3d scale(sx, sy, sz);

  // subtract mean and then scale
  vout.resize(vin.size());
  for(size_t i = 0; i < vin.size(); ++i)
  {
    vout[i](0) = (vin[i] - mean)(0) * sx;
    vout[i](1) = (vin[i] - mean)(1) * sy;
    vout[i](2) = (vin[i] - mean)(2) * sz;

    vout[i] += mean;
  }
}

void leatherman::scaleVertices(const std::vector<geometry_msgs::Point> &vin, double sx, double sy, double sz, std::vector<geometry_msgs::Point> &vout)
{
  std::vector<Eigen::Vector3d> evin(vin.size()), evout;
  for(size_t p = 0; p < vin.size(); ++p)
  {
    evin[p](0) = vin[p].x;
    evin[p](1) = vin[p].y;
    evin[p](2) = vin[p].z;
  }

  scaleVertices(evin, sx, sy, sz, evout);

  vout.resize(evout.size());
  for(size_t p = 0; p < evout.size(); ++p)
  {
    vout[p].x = evout[p](0);
    vout[p].y = evout[p](1);
    vout[p].z = evout[p](2);
  }
}

double leatherman::getColladaFileScale(std::string resource)
{
  static std::map<std::string, float> rescale_cache;

  // Try to read unit to meter conversion ratio from mesh. Only valid in Collada XML formats.
  TiXmlDocument xmlDoc;
  float unit_scale(1.0);
  resource_retriever::Retriever retriever;
  resource_retriever::MemoryResource res;
  try
  {
    res = retriever.get(resource);
  }
  catch (resource_retriever::Exception& e)
  {
    ROS_ERROR("%s", e.what());
    return unit_scale;
  }

  if (res.size == 0)
  {
    return unit_scale;
  }


  // Use the resource retriever to get the data.
  const char * data = reinterpret_cast<const char * > (res.data.get());
  xmlDoc.Parse(data);

  // Find the appropriate element if it exists
  if(!xmlDoc.Error())
  {
    TiXmlElement * colladaXml = xmlDoc.FirstChildElement("COLLADA");
    if(colladaXml)
    {
      TiXmlElement *assetXml = colladaXml->FirstChildElement("asset");
      if(assetXml)
      {
        TiXmlElement *unitXml = assetXml->FirstChildElement("unit");
        if (unitXml && unitXml->Attribute("meter"))
        {
          // Failing to convert leaves unit_scale as the default.
          if(unitXml->QueryFloatAttribute("meter", &unit_scale) != 0)
            ROS_WARN_STREAM("getMeshUnitRescale::Failed to convert unit element meter attribute to determine scaling. unit element: " << *unitXml);
        }
      }
    }
  }
  return unit_scale;
}

bool leatherman::getLinkMesh(std::string urdf, std::string name, bool collision, std::string &mesh_resource, geometry_msgs::PoseStamped &pose)
{
  urdf::Model model;
  if(!model.initString(urdf))
  {
    ROS_ERROR("Something is wrong with the URDF.");
    return false;
  }

  boost::shared_ptr<const urdf::Link> link = model.getLink(name);
  if(link == NULL)
  {
    ROS_ERROR("Failed to find link '%s' in URDF.", name.c_str());
    return false;
  }
  if(link->collision == NULL)
  {
    ROS_ERROR("Failed to find collision field for link '%s' in URDF.", link->name.c_str());
    return false;
  }
  if(link->collision->geometry == NULL)
  {
    ROS_ERROR("Failed to find geometry for link '%s' in URDF. (group: %s)", name.c_str(), link->collision->group_name.c_str());
    return false;
  }

  boost::shared_ptr<const urdf::Geometry> geom;
  if(collision)
  {
    geom = link->visual->geometry;
    pose.pose.position.x = link->visual->origin.position.x;
    pose.pose.position.y = link->visual->origin.position.y;
    pose.pose.position.z = link->visual->origin.position.z;
    link->visual->origin.rotation.getQuaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  }
  else
  {
    geom = link->collision->geometry;
    pose.pose.position.x = link->collision->origin.position.x;
    pose.pose.position.y = link->collision->origin.position.y;
    pose.pose.position.z = link->collision->origin.position.z;
    link->collision->origin.rotation.getQuaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  }

  if(geom->type != urdf::Geometry::MESH)
  {
    ROS_ERROR("Failed because geometry is not a mesh.");
    return false;
  }
  urdf::Mesh* mesh = (urdf::Mesh*) geom.get();
  mesh_resource = mesh->filename;
  pose.header.frame_id = link->parent_joint->child_link_name;
  return true;
}

