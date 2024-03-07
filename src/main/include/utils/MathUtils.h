// Utilites to accomplish specific mathamatical funtions 
// Based off of team 1706 2022 robot code - https://github.com/rr1706/2022-Main/tree/main

#pragma once

#include <frc/geometry/Pose2d.h>

namespace MathUtils {
  /**
   * Takes an input value and squares it, but retains the sign. IE negative
   * numbers will remain negative.
   * 
   * @param input is the number to perform the transform on
   * @return the transformed input value
  */
  double SignedSquare(double input);

  /**
   * Finds transformation of robot in relation to the goal. 
   * 
   * @param robotPose is the position of the robot on the field
   * @return the transformation of the robot
  */
  frc::Translation2d TranslationToGoal(frc::Pose2d robotPose);

  /**
   * Turns translation2d given by TranslationToGoal to distance between the robot and 
   * goal in meters.
   * 
   * @param robotTransformation is the transformation given by TranslationToGoal
   * @return the distance to the goal in meters
  */
  double RobotDistanceToGoal(frc::Pose2d robotPose);

  /**
   * Finds the angle of the robot in relation to the goal. 
   * 
   * @param targetTranslation is the position of the robot on the field
   * @return the rotation of the robot
  */
  frc::Rotation2d AngleToGoal(frc::Translation2d targetTranslation);
};
