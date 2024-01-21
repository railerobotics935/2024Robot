
#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <Constants.h>

#include "subsystems/ApriltagSensor.h"

/**
 * List of things to do to keep organized
 * 
 * other things also in robot
 * 
 * DONE: Update robot position with one Apriltag
 *    DONE: Pull apriltag array from NT
 *    DONE: Update Odometry with the new tranlsation 3d values
 * DONE: Adjust for multiple Apriltags
 * TODO: Test Accuracy/speed of detection
 * TODO: Classify when to rely more on apriltag position
 *    - Zones
 *    - Distance from apriltag
*/
ApriltagSensor::ApriltagSensor(std::string cameraName) {
	
	// Set the camera name to identify whitch camera to look at in NT
	m_cameraName = cameraName;

	auto nt_inst = nt::NetworkTableInstance::GetDefault();
	auto nt_table = nt_inst.GetTable("SmartDashboard");

  // Cycle through each tag ID and get entry for each - status, depth and pose
  char s_tableEntryPath[32];
  for (uint8_t i = 0; i < MAX_NUM_TAGS; i++) {
    // Status holds a sting, either "TRACKED" or "LOST"
    sprintf(s_tableEntryPath, "%s/Tag[%d]/Status", m_cameraName.c_str(), i);
    nte_status[i] = nt_table->GetEntry(s_tableEntryPath);

    // Depth holds an array of doubles for the x, y and z 
    sprintf(s_tableEntryPath, "%s/Tag[%d]/Depth", m_cameraName.c_str(), i);
    nte_depth[i] = nt_table->GetEntry(s_tableEntryPath);

    // Depth holds an array of doubles for the x, y and z, then rotx, roty and rotz
    sprintf(s_tableEntryPath, "%s/Tag[%d]/Pose", m_cameraName.c_str(), i);
    nte_pose[i] = nt_table->GetEntry(s_tableEntryPath);
  }

}

frc::Pose2d ApriltagSensor::GetApriltagRelativePose(int tag) {
  // Grab Pose3d values in an vector
  std::vector<double> poseArr = nte_pose[tag].GetDoubleArray(std::vector<double>());
  
  // Default to zeros
  frc::Pose2d returnPose = frc::Pose2d((units::meter_t)0.0, (units::meter_t)0.0, (units::radian_t)0.0);

  // If tracked, grab numbers
  if (nte_status[tag].GetString("LOST") == "TRACKED") {
    // Pose2d made up of x and y tranlation with z rotation
    returnPose = frc::Pose2d((units::meter_t)poseArr[0], (units::meter_t)poseArr[1], (units::radian_t)poseArr[5]);
    returnPose.TransformBy(CameraConstats::kFrontCameraTransform2d);
  }

  return returnPose;
}

frc::Transform2d ApriltagSensor::GetApriltagRelativeTransformation(int tag) {
  // Grab Pose3d values in an vector
  std::vector<double> poseArr = nte_pose[tag].GetDoubleArray(std::vector<double>());
  
  // Default to zeros
  frc::Transform2d returnTransformation{(units::meter_t)0.0, (units::meter_t)0.0, (units::radian_t)0.0};

  // If tracked, grab numbers
  if (nte_status[tag].GetString("LOST") == "TRACKED") {
    // Transform2d made up of x and y tranlation with z rotation
    returnTransformation = frc::Transform2d((units::meter_t)poseArr[0], (units::meter_t)poseArr[1], (units::radian_t)poseArr[5]);
    returnTransformation.operator+(CameraConstats::kFrontCameraTransform2d);
  }

  return returnTransformation;
}

frc::Pose2d ApriltagSensor::GetFieldRelativePose(int tag) {
  // Create pose2d to return
  std::vector<double> poseArr = nte_pose[tag].GetDoubleArray(std::vector<double>());
  
  // Default to zeros
  frc::Pose2d returnPose = frc::Pose2d((units::meter_t)0.0, (units::meter_t)0.0, (units::radian_t)0.0);

  // If tracked, grab numbers
  if (nte_status[tag].GetString("LOST") == "TRACKED") {
    // Rotate the input pose by the angle of the tag to make it oriented like the field
    //frc::Pose2d fieldCorrectedInputPose = GetApriltagRelativePose(tag).RotateBy((units::radian_t)(2 * std::numbers::pi) - m_fieldLayout.GetTagPose(tag).value().ToPose2d().Rotation().Radians());

    // Grab the Pose2d of the apriltag
    frc::Pose2d tagPoseOnField = m_fieldLayout.GetTagPose(tag).value().ToPose2d();

    // Transform the field pose by the robot relative pose
    returnPose = tagPoseOnField.TransformBy(GetApriltagRelativeTransformation(tag));
  }

  return returnPose;
}
