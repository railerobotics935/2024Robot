
#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <Constants.h>
#include <frc/geometry/CoordinateSystem.h>

#include "subsystems/sensors/OakDLiteSensor.h"


OakDLiteSensor::OakDLiteSensor(std::string cameraName, frc::Pose3d cameraPose3d, frc::SwerveDrivePoseEstimator<4>* poseEstimator) {
	
	// Set the camera name to identify whitch camera to look at in NT
	m_cameraName = cameraName;
  m_cameraPose3d = cameraPose3d;
  m_poseEstimator = poseEstimator;

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

frc::Translation2d OakDLiteSensor::GetFieldRelativeTranslation(int object) {
  // Grab Translation3d values in an vector
  std::vector<double> translationArr = nte_location[object].GetDoubleArray(std::vector<double>());

  // Create Transform3d object for object position relative to robot
  frc::Translation3d rawTranslation{(units::meter_t)translationArr[0], (units::meter_t)translationArr[1], (units::meter_t)translationArr[2]};
  
  // Convert translation into standard for the robot
  frc::Translation3d convertedTranslation = frc::CoordinateSystem::Convert(rawTranslation, 
                                frc::CoordinateSystem::EDN(), 
                                frc::CoordinateSystem::NWU());
  
  // Correcting and adding translations to get final translation of the object 
  return convertedTranslation.ToTranslation2d().operator+(m_cameraPose3d.ToPose2d().Translation()).RotateBy(-m_poseEstimator->GetEstimatedPosition().Rotation()).operator+(m_poseEstimator->GetEstimatedPosition().Translation());
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