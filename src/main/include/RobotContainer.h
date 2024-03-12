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
#include <frc/PowerDistribution.h>

#include "Constants.h"
#include "utils/SwerveUtils.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/StagerSubsystem.h"
#include "subsystems/ClimberSubsystem.h"

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

  /**
   * @return The command for autonomous
  */
  frc2::CommandPtr GetAutonomousCommand();

 private:
  /**
   * Convigures button bindings for commands
  */
  void ConfigureButtonBindings();

  // The driver and operator controllers
  frc::XboxController m_driveController{OIConstants::kDriverControllerPort};
  frc::XboxController m_operatorController{OIConstants::kOperatorControllerPort};

  frc::PowerDistribution m_revPDH{1, frc::PowerDistribution::ModuleType::kRev};
  // Variables 
  bool isFieldRelative = true;

  // The robot's subsystems
  DriveSubsystem m_drive;
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;
  StagerSubsystem m_stager;
  ClimberSubsystem m_leftClimber{ClimberConstants::LeftClimber::kID, ClimberConstants::LeftClimber::kLimitSwitchPort};
  ClimberSubsystem m_rightClimber{ClimberConstants::RightClimber::kID, ClimberConstants::RightClimber::kLimitSwitchPort};

  // Sendable chooser for auto
  frc::SendableChooser<std::string> m_autoChooser;

  // Auto options coresponding to the name of the autos                                             
  std::string m_forward1m = "Forward1m";
  std::string m_left1m = "Left1m";
  std::string m_trapezoidTest = "TrapazoidTest";
  std::string m_testIntake = "TestIntake";

};
