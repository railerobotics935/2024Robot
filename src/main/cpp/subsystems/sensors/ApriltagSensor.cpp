
#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <Constants.h>
#include <frc/geometry/CoordinateSystem.h>
#include <frc/Timer.h>

#include "subsystems/sensors/ApriltagSensor.h"


ApriltagSensor::ApriltagSensor(std::string cameraName, frc::Pose3d cameraPose3d) {
	
	// Set the camera name to identify whitch camera to look at in NT
	m_cameraName = cameraName;
  m_cameraPose3d = cameraPose3d;
  m_cameraTransform2d = {cameraPose3d.ToPose2d().Translation(), cameraPose3d.ToPose2d().Rotation()};

	auto nt_inst = nt::NetworkTableInstance::GetDefault();
	auto nt_table = nt_inst.GetTable("SmartDashboard");

  // Cycle through each tag ID and get entry for each - status and pose
  char s_tableEntryPath[32]; 
  for (uint8_t i = 0; i < MAX_NUM_TAGS; i++) {
    // Status holds a sting, either "TRACKED" or "LOST"
    sprintf(s_tableEntryPath, "%s/Tag[%d]/Status", m_cameraName.c_str(), i);
    nte_status[i] = nt_table->GetEntry(s_tableEntryPath);

    // Depth holds an array of doubles for the x, y and z, then rotx, roty and rotz
    sprintf(s_tableEntryPath, "%s/Tag[%d]/Pose", m_cameraName.c_str(), i);
    nte_pose[i] = nt_table->GetEntry(s_tableEntryPath);
  }

  // Latency from time camera picks up image to when the pi published the data
  sprintf(s_tableEntryPath, "%s/Latency/Apriltag", m_cameraName.c_str());
  nte_latency = nt_table->GetEntry(s_tableEntryPath);

    // Latency from time camera picks up image to when the pi published the data
  sprintf(s_tableEntryPath, "%s/Latency/Final", m_cameraName.c_str());
  nte_finalLatency = nt_table->GetEntry(s_tableEntryPath);

}

frc::Pose3d ApriltagSensor::GetFieldRelativePose(int tag) {
  // Grab Pose3d values in an vector
  m_poseArr = nte_pose[tag].GetDoubleArray(std::vector<double>());

  // Create Transform3d object for tag position relative to robot
  m_rawTranslation = {(units::meter_t)m_poseArr[0], (units::meter_t)m_poseArr[1], (units::meter_t)m_poseArr[2]};
  m_rawRotation = {(units::radian_t)m_poseArr[3], (units::radian_t)m_poseArr[4], (units::radian_t)m_poseArr[5]};
  m_rawPose = {m_rawTranslation, m_rawRotation};
  
  // Correct rotation so the robot is rotated, not the apriltag
  m_correctedRotation = {(units::radian_t)m_poseArr[5], (units::radian_t)m_poseArr[3], (units::radian_t)(m_poseArr[4] + std::numbers::pi)}; // correct axis manualy and add pi to the rotation axis to face tag instead

  // Convert translation into standard for the robot
  m_convertedTranslation = frc::CoordinateSystem::Convert(m_rawPose.Translation().RotateBy(m_rawPose.Inverse().Rotation()), 
                                frc::CoordinateSystem::EDN(), 
                                frc::CoordinateSystem::NWU());
  
  // Final Transformation
  return m_fieldLayout.GetTagPose(tag).value().TransformBy(frc::Transform3d{m_convertedTranslation, m_correctedRotation}).TransformBy(frc::Transform3d{m_cameraPose3d.Translation(), m_cameraPose3d.Rotation()});
}

wpi::array<double, 3> ApriltagSensor::GetStandardDeviations(int tag) {
  // Grab Pose3d values in an vector
  m_poseArr = nte_pose[tag].GetDoubleArray(std::vector<double>());
  
  // use the raw distance to get the information
  double standardDeviation = CameraConstants::GetStandardDeviationFromDistance((double)m_poseArr[2]);

  return wpi::array<double, 3>{standardDeviation, standardDeviation, standardDeviation};
}

bool ApriltagSensor::TagIsTracked(int tag) {
  // If tag is traked, return true, else return false
  if (nte_status[tag].GetString("LOST") == "TRACKED")
    return true;
  else
    return false;
}

units::second_t ApriltagSensor::GetTimestamp(int tag) {
  units::second_t timestamp = (m_timer.GetFPGATimestamp()) - (units::second_t)nte_latency.GetDouble(360.0) - (units::second_t)0.05;
  nte_finalLatency.SetDouble((double)timestamp);
  return (units::second_t)timestamp;
}

bool ApriltagSensor::HasNewData(int tag) {
  return true;
  
  if (nte_pose[tag].GetLastChange() - m_prevLatency != 0) {
    m_prevLatency = nte_pose[tag].GetLastChange();
    return true;
  }
  else
    return false;
}