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
   * Finds distance between goal of the alliance you're on and the robot. 
   * 
   * @param robotPose is the position of the robot on the field
   * @return the distance from the goal in meters
  */
  double TranslationToGoal(frc::Pose2d robotPose);

  
};

