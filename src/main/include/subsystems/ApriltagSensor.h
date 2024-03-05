

#pragma once

#include <networktables/NetworktableEntry.h>
#include <networktables/networkTableInstance.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/Filesystem.h>
#include <wpi/fs.h>
#include <wpi/array.h>

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
   * @param cameraPose3d The 3d position of the camera 
  */
  ApriltagSensor (std::string cameraName, frc::Pose3d cameraPose3d);

  /**
   * @param tag The ID number for the apriltag wanted to identify
   * @return The Field Relative Pose3d
  */
  frc::Pose3d GetFieldRelativePose(int tag);

  /**
   * @param tag the ID number for the apriltag wanted to identify
   * @return An array of standard deviations scaled by the distance form the shooter
  */
  wpi::array<double, 3> GetStandardDeviations(int tag);

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
  nt::NetworkTableEntry nte_finalLatency;

  std::string m_cameraName;
  frc::Pose3d m_cameraPose3d;
  frc::Transform2d m_cameraTransform2d;
  
  // Create path to deploy directory
  fs::path deployDirectory{frc::filesystem::GetDeployDirectory() + "/2024-crescendo.json"};

  // Field layout to get apriltag pose
  frc::AprilTagFieldLayout m_fieldLayout{deployDirectory.string()};
};