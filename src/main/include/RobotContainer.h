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

#include "commands/drive/DriveWithController.h"

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
  ClimberSubsystem m_climber;

  DriveWithController m_driveCommand{&m_drive, &m_driveController};
  
  // Sendable chooser for auto
  frc::SendableChooser<std::string> m_autoChooser;

  // Auto options coresponding to the name of the autos                                             
  std::string m_speaker21 = "Speaker21";
  std::string m_amp12 = "Amp12";
  std::string m_speaker21Far = "Speaker21Far";
  std::string m_speaker241 = "Speaker241";
  std::string m_speaker213 = "Speaker213";
  std::string m_speaker23 = "Speaker23";
  std::string m_amp1 = "Amp1";
  std::string m_shootOne = "ShootOne";
  std::string m_source3 = "Source3";
  std::string m_source32 = "Source32";
  std::string m_speaker2 = "Speaker2";  
  std::string m_sourceTravel = "SourceTravel";  

};
