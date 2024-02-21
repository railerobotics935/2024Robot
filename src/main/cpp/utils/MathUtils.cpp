
#include <math.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>


#include "utils/MathUtils.h"

double MathUtils::SignedSquare(double input) {
  if (input < 0.0)
    return -std::pow(input, 2);
  else
    return std::pow(input, 2);
}

double MathUtils::TranslationToGoal(frc::Pose2d robotPose) {
  // Initialize variables
  frc::AprilTagFieldLayout m_fieldLayout; 
  frc::Pose2d centerOfSpeaker;
  
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
    // Get position of center of blue speaker
    centerOfSpeaker = m_fieldLayout.GetTagPose(7).value().ToPose2d(); 
  } else {
    // Get position of center of red speaker
    centerOfSpeaker = m_fieldLayout.GetTagPose(4).value().ToPose2d(); 
  }

  // Find transformation of robot
  frc::Transform2d robotTransformation = robotPose.operator-(centerOfSpeaker);

  // Finding the distance between the robot and center of goal in meters
  double robotDistanceToGoal = std::sqrt(std::pow((double)robotTransformation.X(), 2) + std::pow((double)robotTransformation.Y(), 2));
  return robotDistanceToGoal;
}