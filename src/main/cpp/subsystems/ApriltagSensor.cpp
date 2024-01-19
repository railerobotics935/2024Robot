
#include "subsystems/ApriltagSensor.h"
#include <vector>

/**
 * List of things to do to keep organized
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

frc::Pose2d ApriltagSensor::GetRobotRelativePose(int tag) {
  // Create pose2d to return
  std::vector<double> poseArr = nte_pose[tag].GetDoubleArray(std::vector<double>());
  
  // Default to zeros
  frc::Pose2d returnPose = frc::Pose2d((units::meter_t)0.0, (units::meter_t)0.0, (units::radian_t)0.0);

  // If tracked, grab numbers
  if (nte_status[tag].GetString("LOST") == "TRACKED") {
    // Pose2d made up of x and y tranlation with z rotation
    returnPose = frc::Pose2d((units::meter_t)poseArr[0], (units::meter_t)poseArr[1], (units::radian_t)poseArr[5]);
  }

  return returnPose;
}
