
#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <Constants.h>


#include "subsystems/ApriltagSensor.h"


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

}

frc::Pose3d ApriltagSensor::GetRawPose3d(int tag) {
  // Grab Pose3d values in an vector
  std::vector<double> poseArr = nte_pose[tag].GetDoubleArray(std::vector<double>());
  
  // Put array into translation3d and rotation3d
  frc::Translation3d tagTranslation{(units::meter_t)poseArr[2], -(units::meter_t)poseArr[0], -(units::meter_t)poseArr[1]};
  frc::Rotation3d tagRotation{(units::radian_t)poseArr[5], (units::radian_t)poseArr[3], (units::radian_t)poseArr[4]};

  // Combine Translation3d and Rotation3d to make a Pose3d
  frc::Pose3d returnPose{tagTranslation, tagRotation};

  return returnPose;
}


frc::Pose2d ApriltagSensor::GetApriltagRelativePose(int tag) {

  // Default to zeros
  frc::Pose2d returnPose = frc::Pose2d((units::meter_t)0.0, (units::meter_t)0.0, (units::radian_t)0.0);

  // If tracked, return NT values
  if (nte_status[tag].GetString("LOST") == "TRACKED") {
    // GetPose2d from Pose3d
    returnPose = GetRawPose3d(tag).ToPose2d().RotateBy((units::radian_t)std::numbers::pi);

    // Transform by camera Pose2d
    returnPose.TransformBy(m_cameraTransform2d);
  }

  return returnPose;
}

frc::Transform2d ApriltagSensor::GetApriltagRelativeTransformation(int tag) {

  // Default to zeros
  frc::Transform2d returnTransformation{(units::meter_t)0.0, (units::meter_t)0.0, (units::radian_t)0.0};

  // If tracked, return NT values
  if (nte_status[tag].GetString("LOST") == "TRACKED") {
    // Grab Translation2d and Rotation2d componets of the Pose to make the transformation from the tag
    returnTransformation = frc::Transform2d(GetRawPose3d(tag).ToPose2d().Translation(), GetRawPose3d(tag).ToPose2d().RotateBy((units::radian_t)std::numbers::pi).Rotation());

    // Add the transformation from the camera Pose2d together
    returnTransformation.operator+(m_cameraTransform2d);
  }

  return returnTransformation;
}

frc::Pose2d ApriltagSensor::GetFieldRelativePose(int tag) {

  // Default to zeros
  frc::Pose2d returnPose = frc::Pose2d((units::meter_t)0.0, (units::meter_t)0.0, (units::radian_t)0.0);

  // If tracked, grab numbers
  if (nte_status[tag].GetString("LOST") == "TRACKED") {

    // Grab the Pose2d of the apriltag
    frc::Pose2d tagPoseOnField = m_fieldLayout.GetTagPose(tag).value().ToPose2d();

    // Transform the field pose by the robot relative pose
    returnPose = tagPoseOnField.TransformBy(GetApriltagRelativeTransformation(tag));
  }

  return returnPose;
}

bool ApriltagSensor::TagIsTracked(int tag) {
  // If tag is traked, return true, else return false
  if (nte_status[tag].GetString("LOST") == "TRACKED")
    return true;
  else
    return false;
}

units::second_t ApriltagSensor::GetTimestamp(int tag) {
  return (units::second_t)(nte_pose[tag].GetLastChange() / 1000000.0) - (units::second_t)nte_latency.GetDouble(360.0);
}