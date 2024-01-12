// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/filter/SlewRateLimiter.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"


/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  // The driver's controller
  frc::XboxController m_driveController{IOConstants::kDriverControllerPort};

  // Variables 
  bool isFieldRelative = true;

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;

  // Slew rate limiters to make joystick inputs more gentle; 1/2 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xSpeedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_ySpeedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{2 / 1_s};

  void ConfigureButtonBindings();

  // Sendable chooser for auto
  frc::SendableChooser<std::string> m_autoChooser;

  // Auto options coresponding to the name of the autos                                             
  std::string m_defaultAuto = "Figure8";
  std::string m_otherAuto = "OtherAuto";
};
