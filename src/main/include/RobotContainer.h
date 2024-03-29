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

#include "commands/auto/SetCloseShooterSpeeds.h"
#include "commands/auto/SetFarShooterSpeeds.h"
#include "commands/auto/StageForShooting.h"
#include "commands/auto/StopEverything.h"
#include "commands/auto/StopStager.h"
#include "commands/drive/DriveWithController.h"
#include "commands/drive/DriveFacingGoal.h"
#include "commands/drive/SlowDrive.h"
#include "commands/intake/ManualStager.h"
#include "commands/intake/SimpleIntake.h"
#include "commands/intake/SmartIntake.h"
#include "commands/intake/SmartOuttake.h"
#include "commands/intake/StopIntake.h"
#include "commands/shooter/DefaultShooter.h"
#include "commands/shooter/ManualNteShooter.h"
#include "commands/shooter/SmartShooter.h"
#include "commands/shooter/ManualCloseShoot.h"
#include "commands/shooter/ManualFarShoot.h"
#include "commands/climber/ExtendClimber.h"
#include "commands/climber/RetractClimber.h"
#include "commands/climber/StopClimber.h"
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

  SetCloseShooterSpeeds m_setCloseShooterSpeeds{&m_shooter};
  SetFarShooterSpeeds m_setFarShooterSpeeds{&m_shooter};
  StageForShooting m_stageForShooting{&m_stager};
  StopEverything m_stopEverything{&m_stager, &m_shooter};
  StopStager m_stopStager{&m_stager};
  DriveWithController m_driveWithController{&m_drive, &m_driveController};
  DriveFacingGoal m_driveFacingGoal{&m_drive, &m_driveController};
  SlowDrive m_slowDrive{&m_drive, &m_driveController};
  ManualStager m_manualStager{&m_stager, &m_operatorController};
  SimpleIntake m_simpleIntake{&m_intake};
  SmartIntake m_smartIntake{&m_intake, &m_stager};
  SmartOuttake m_smartOuttake{&m_intake, &m_stager};
  StopIntake m_stopIntake{&m_intake};
  DefaultShooter m_defaultShooter{&m_shooter};
  ManualNteShooter m_manualNteShooter{&m_shooter, &m_operatorController};
  SmartShooter m_smartShooting{&m_shooter, &m_drive, &m_driveController, &m_operatorController};
  ManualCloseShoot m_manualCloseShoot{&m_shooter};
  ManualFarShoot m_manualFarShoot{&m_shooter};
  ExtendClimber m_extendClimber{&m_climber};
  RetractClimber m_retractClimber{&m_climber};
  StopClimber m_stopClimber{&m_climber};
  DriveFacingGoal m_driveFacingGoal{&m_drive, &m_driveController};

};
