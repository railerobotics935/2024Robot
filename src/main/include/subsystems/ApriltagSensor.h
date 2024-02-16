

#pragma once

#include <networktables/NetworktableEntry.h>
#include <networktables/networkTableInstance.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/Filesystem.h>
#include <wpi/fs.h>

#include <string.h>

#define MAX_NUM_TAGS 16


class ApriltagSensor {
public:
    /**
   * ApriltagSensor is meant to be implemnted as any other sensor for the robot
   * but simply takes information from Network tables and oragnizes it for use 
   * in the robot code
   * 
   * @param cameraName The camera name from network tables
  */
  ApriltagSensor (std::string cameraName, frc::Pose3d cameraPose3d);

  /**
   * @param tag The ID number for the apriltag wanted to identify
   * @return The raw Pose3d from the camera
  */
  frc::Pose3d GetRawPose3d(int tag);

  /**
   * @param tag The ID number for the apriltag wanted to identify
   * @return The Apriltag Relative Pose2d of the center of the robot
  */
  frc::Pose2d GetApriltagRelativePose(int tag);

  /**
   * @param tag The ID number for the apriltag wanted to identify
   * @return The Apriltag Relative transform2d of the center of the robot
  */
  frc::Transform2d GetApriltagRelativeTransformation(int tag);

  /**
   * @param tag The ID number for the apriltag wanted to identify
   * @return The Field Relative Pose2d
  */
  frc::Pose2d GetFieldRelativePose(int tag);

  /**
   * @return If the tag is tracked
  */
  bool TagIsTracked(int tag);

  /**
   * @param tag The ID number for the apriltag wanted to identify
   * @return The Timestamp of a pose
  */
  units::second_t GetTimestamp(int tag);

private:
  // Declare Network table entrys for apriltag pos
  nt::NetworkTableEntry nte_status[MAX_NUM_TAGS];
  nt::NetworkTableEntry nte_pose[MAX_NUM_TAGS];
  nt::NetworkTableEntry nte_latency;

  std::string m_cameraName;
  frc::Pose3d m_cameraPose3d;
  frc::Transform2d m_cameraTransform2d;
  // Create path to deploy directory
  fs::path deployDirectory{frc::filesystem::GetDeployDirectory() + "/2024-crescendo.json"};

  // Field layout to get apriltag pose
  frc::AprilTagFieldLayout m_fieldLayout{deployDirectory.string()};
};