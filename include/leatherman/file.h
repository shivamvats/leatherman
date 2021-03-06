#ifndef _LEATHERMAN_FILE_
#define _LEATHERMAN_FILE_

// standard includes
#include <string>
#include <vector>

// system includes
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <trajectory_msgs/JointTrajectory.h>

namespace leatherman
{
  std::string getTime();

  bool createFolder(std::string name);

  bool getFolderContents(std::string folder_name, std::vector<std::string>& files);

  bool writeJointTrajectoryToFile(FILE** file, const trajectory_msgs::JointTrajectory &traj);

  bool writePointsToFile(std::string filename, const std::vector<Eigen::Vector3d> &pts);

  bool readPointsInFile(std::string filename, std::vector<Eigen::Vector3d> &pts);

  std::string getFilenameFromPath(std::string path, bool remove_extension=false);

  std::string getPathWithoutFilename(std::string path);

  std::string getExtension(std::string filename);

  std::string replaceExtension(std::string filename, std::string extension);

  bool getSystemPathFromROSPath(std::string ros_path, std::string &system_path);
}

#endif
