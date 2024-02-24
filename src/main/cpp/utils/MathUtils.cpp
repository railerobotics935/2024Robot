
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

frc::Translation2d MathUtils::TranslationToGoal(frc::Pose2d robotPose) {
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

  // Find Translation of robot
  frc::Translation2d robotTranslation = robotPose.operator-(centerOfSpeaker).Translation();
  return robotTranslation;
/*
  // Finding the distance between the robot and center of goal in meters
  double robotDistanceToGoal = std::sqrt(std::pow((double)robotTranslation.X(), 2) + std::pow((double)robotTranslation.Y(), 2));
  return robotDistanceToGoal;
*/
}

double MathUtils::RobotDistanceToGoal(frc::Pose2d robotPose) {
  //Find the distance between the robot and the goal
  double robotDistanceToGoal = std::sqrt(std::pow((double)robotPose.X(), 2) + std::pow((double)robotPose.Y(), 2));
  return robotDistanceToGoal;
}