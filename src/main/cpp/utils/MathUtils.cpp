
#include <math.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <wpi/fs.h>
#include <frc/Filesystem.h>
#include <iostream>

#include "utils/MathUtils.h"

double MathUtils::SignedSquare(double input) {
  if (input < 0.0)
    return -std::pow(input, 2);
  else
    return std::pow(input, 2);
}

frc::Translation2d MathUtils::TranslationToGoal(frc::Pose2d robotPose) {
  std::cout << "EXPENSIVE FUNCTION CALLED\r\n";
  /*
  // Create path to deploy directory
  fs::path deployDirectory{frc::filesystem::GetDeployDirectory() + "/2024-crescendo.json"};
  // Initialize variables
  frc::AprilTagFieldLayout fieldLayout{deployDirectory.string()}; 
  frc::Pose2d centerOfSpeaker{};

  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
    // Get position of center of blue speaker
    centerOfSpeaker = fieldLayout.GetTagPose(7).value().ToPose2d(); 
  } else {
    // Get position of center of red speaker
    centerOfSpeaker = fieldLayout.GetTagPose(4).value().ToPose2d(); 
  }
  */
  
  // Find Translation of robot
  return frc::Translation2d{};//robotPose.operator-(centerOfSpeaker).Translation();
}

double MathUtils::RobotDistanceToGoal(frc::Pose2d robotPose) {
  //Find the distance between the robot and the goal
  frc::Translation2d translationToGoal = TranslationToGoal(robotPose);
  double robotDistanceToGoal = std::sqrt(std::pow((double)translationToGoal.X(), 2) + std::pow((double)translationToGoal.Y(), 2));
  return robotDistanceToGoal;
}

frc::Rotation2d MathUtils::AngleToGoal(frc::Translation2d targetTranslation) {
  // do math
  return frc::Rotation2d{(units::radian_t)std::atan(((double)targetTranslation.Y())/((double)targetTranslation.X()))}.operator+((units::degree_t)180.0);
}