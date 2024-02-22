
#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <Constants.h>
#include <frc/geometry/CoordinateSystem.h>

#include "subsystems/OakDLiteSensor.h"


OakDLiteSensor::OakDLiteSensor(std::string cameraName, frc::Pose3d cameraPose3d, frc::SwerveDrivePoseEstimator<4>* poseEstimator) {
	
	// Set the camera name to identify whitch camera to look at in NT
	m_cameraName = cameraName;
  m_cameraPose3d = cameraPose3d;
  m_poseEstimator = poseEstimator;
  m_cameraTransform2d = {cameraPose3d.ToPose2d().Translation(), cameraPose3d.ToPose2d().Rotation()};

	auto nt_inst = nt::NetworkTableInstance::GetDefault();
	auto nt_table = nt_inst.GetTable("SmartDashboard");

  // Cycle through each tag ID and get entry for each - status and pose
  char s_tableEntryPath[32]; 
  for (uint8_t i = 0; i < MAX_NUM_OBJECTS; i++) {
    // Status holds a string, either "TRACKED" or "LOST"
    sprintf(s_tableEntryPath, "%s/Object[%d]/Status", m_cameraName.c_str(), i);
    nte_status[i] = nt_table->GetEntry(s_tableEntryPath);

    // Location holds an array of doubles for the x, y, and z of the object
    sprintf(s_tableEntryPath, "%s/Object[%d]/Location", m_cameraName.c_str(), i);
    nte_location[i] = nt_table->GetEntry(s_tableEntryPath);

    // Type holds a string, either "robot" or "note"
    sprintf(s_tableEntryPath, "%s/Object[%d]/Type", m_cameraName.c_str(), i);
    nte_type[i] = nt_table->GetEntry(s_tableEntryPath);
  }

  // Latency from time camera picks up image to when the pi published the data
  sprintf(s_tableEntryPath, "%s/Latency/ObjectDetect", m_cameraName.c_str());
  nte_latency = nt_table->GetEntry(s_tableEntryPath);

}

frc::Pose3d OakDLiteSensor::GetFieldRelativePose(int object) {
  // Grab Pose3d values in an vector
  std::vector<double> poseArr = nte_location[object].GetDoubleArray(std::vector<double>());

  // Create Transform3d object for tag position relative to robot
  frc::Translation3d rawTranslation{(units::meter_t)poseArr[0], (units::meter_t)poseArr[1], (units::meter_t)poseArr[2]};
  frc::Rotation3d rawRotation{(units::radian_t)poseArr[3], (units::radian_t)poseArr[4], (units::radian_t)poseArr[5]};
  frc::Transform3d rawPose{rawTranslation, rawRotation};
  
  // Correct rotation so the robot is rotated, not the apriltag
  frc::Rotation3d correctedRotation{(units::radian_t)poseArr[5], (units::radian_t)poseArr[3], (units::radian_t)(poseArr[4] + std::numbers::pi)}; // correct axis manualy and add pi to the rotation axis to face tag instead

  // Convert translation into standard for the robot
  frc::Translation3d convertedTranslation = frc::CoordinateSystem::Convert(rawPose.Translation().RotateBy(rawPose.Inverse().Rotation()), 
                                frc::CoordinateSystem::EDN(), 
                                frc::CoordinateSystem::NWU());
  
  // transform by the tag position on the field and the camera location on the robot
  frc::Pose3d tagPoseOnField{rawTranslation, rawRotation}; //m_fieldLayout.GetTagPose(object).value();
  tagPoseOnField = (tagPoseOnField.TransformBy(frc::Transform3d{convertedTranslation, correctedRotation}).TransformBy(frc::Transform3d{m_cameraPose3d.Translation(), m_cameraPose3d.Rotation()}));
  return tagPoseOnField;
}

bool OakDLiteSensor::ObjectIsTracked(int object) {
  // If object is tracked, return true, else return false
  if (nte_status[object].GetString("LOST") == "TRACKED")
    return true;
  else
    return false;
}

bool OakDLiteSensor::ObjectIsNote(int object) {
  // If object is note, return true, else return false
  if (nte_status[object].GetString("robot") == "note")
    return true;
  else
    return false;
}

units::second_t OakDLiteSensor::GetTimestamp(int object) {
  return (units::second_t)(nte_location[object].GetLastChange() / 1000000.0) - (units::second_t)nte_latency.GetDouble(360.0);
}