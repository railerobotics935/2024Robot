
# pragma once

#include <frc/geometry/rotation2d.h>
#include "frc/kinematics/ChassisSpeeds.h"

namespace RobotUtils
{
  /**
   * Gets the Field Relative Speeds from the robot relative speeds and roation of the robot
   * 
   * @param chassisSpeeds chassisSpeeds of the robot
   * @param rotation Rotation of the robot
   * 
   * @return The field relative speed of the robot in a ChissisSpeed object
  */
  frc::ChassisSpeeds GetFieldRelativeSpeeds(frc::ChassisSpeeds chassisSpeeds, frc::Rotation2d rotation);

  /**
   * Gets the Field Relative Speeds from the robot relative speeds and roation of the robot
   * 
   * @param chassisSpeeds chassisSpeeds of the robot
   * @param rotation Rotation of the robot
   * 
   * @return The field relative speed of the robot in a ChissisSpeed object
  */
  frc::ChassisSpeeds GetFieldRelativeSpeeds(frc::ChassisSpeeds chassisSpeeds, frc::Rotation2d rotation);
} // namespace name
