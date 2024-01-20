

#pragma once

#include <networktables/NetworktableEntry.h>
#include <networktables/networkTableInstance.h>
#include <frc/geometry/Pose2d.h>

#include <string.h>

#define MAX_NUM_TAGS 16

/**
 * ApriltagSensor is ment to be implemnted as any other sensor for the robot
 * but simply takes information from Network tables and oragnizes it for use 
 * in the robot code
 * 
*/
class ApriltagSensor {
public:
  ApriltagSensor(std::string cameraName);

  /**
   * @param tag The ID number for the apriltag wanted to identify
   * @return The Robot Relative Pose2d
  */
  frc::Pose2d GetRobotRelativePose(int tag);

  /**
   * @param tag The ID number for the apriltag wanted to identify
   * @return The Field Relative Pose2d
  */
  frc::Pose2d GetFieldRelativePose(int tag);

private:
  // Declare Network table entrys for apriltag pos
  nt::NetworkTableEntry nte_status[MAX_NUM_TAGS];
  nt::NetworkTableEntry nte_depth[MAX_NUM_TAGS];
  nt::NetworkTableEntry nte_pose[MAX_NUM_TAGS];

  std::string m_cameraName;
};