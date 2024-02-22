
#pragma once

#include <networktables/NetworktableEntry.h>
#include <networktables/networkTableInstance.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <string.h>

#define MAX_NUM_OBJECTS 16

class OakDLiteSensor {
public:
    /**
   * OakDLiteSensor is meant to be implemented as any other sensor for the robot
   * but simply takes information from Network tables and oragnizes it for use 
   * in the robot code
   * 
   * @param cameraName The camera name from network tables
   * @param cameraPose3d The 3d position of the camera 
   * @param poseEstimator Pointer to the poseEstimator of the robot
  */
  OakDLiteSensor (std::string cameraName, frc::Pose3d cameraPose3d, frc::SwerveDrivePoseEstimator<4>* poseEstimator);

  /**
   * @param object The ID number for the object wanted to identify
   * @return The Field Relative Translation 2d
  */
  frc::Translation2d GetFieldRelativeTranslation(int object);

  /**
   * @return True if the object is tracked
  */
  bool ObjectIsTracked(int object);
  
  /**
   * @return True if the object is a note
  */
  bool ObjectIsNote(int object);

  /**
   * @param object The ID number for the object wanted to identify
   * @return The Timestamp of a pose
  */
  units::second_t GetTimestamp(int object);

private:
  // Declare Network table entrys for object pos
  nt::NetworkTableEntry nte_status[MAX_NUM_OBJECTS];
  nt::NetworkTableEntry nte_location[MAX_NUM_OBJECTS];
  nt::NetworkTableEntry nte_type[MAX_NUM_OBJECTS];
  nt::NetworkTableEntry nte_latency;

  // Create local variables
  std::string m_cameraName;
  frc::Pose3d m_cameraPose3d;
  frc::SwerveDrivePoseEstimator<4>* m_poseEstimator;
  frc::Transform2d m_cameraTransform2d;
};