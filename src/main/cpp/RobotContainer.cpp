// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include "pathplanner/lib/commands/PathPlannerAuto.h"
#include "pathplanner/lib/auto/NamedCommands.h"

#include "subsystems/DriveSubsystem.h"
#include "Constants.h"
#include "commands/shooter/AmpShoot.h"

#include "commands/auto/SetSmartShooterSpeeds.h"
#include "commands/shooter/SmartShootWhileMoving.h"

using namespace DriveConstants;
using namespace pathplanner;

/**
 * Idea:
 * 
 * m_drive.SetDefaultCommand(std::move(m_driveCommand));
 *   
 * it says it works, but it hasen't been tested yet. I don't know how different it
 * is, if it is better or not
*/

RobotContainer::RobotContainer() : m_shooter{ShooterConstants::kPitchOffset} {
  m_revPDH.SetSwitchableChannel(true); //-------------------------------------------------------------------------------------

  // Initialize all of your commands and subsystems here
  // Configuring command bindings for pathplanner
  // TODO: Create custom commands for all of these
  NamedCommands::registerCommand("SmartIntake", SmartIntake{&m_intake, &m_stager}.ToPtr());
  NamedCommands::registerCommand("SetShooterSpeeds", SetCloseShooterSpeeds{&m_shooter}.ToPtr());//SetSmartShooterSpeeds{&m_shooter, &m_drive}.ToPtr());
  NamedCommands::registerCommand("SetFarShooterSpeeds", SetFarShooterSpeeds{&m_shooter}.ToPtr());
  NamedCommands::registerCommand("StageForShooting", StageForShooting{&m_stager}.ToPtr());
  NamedCommands::registerCommand("EndShooting", StopStager{&m_stager}.ToPtr());
  NamedCommands::registerCommand("StopEverything", StopEverything{&m_stager, &m_shooter}.ToPtr());
  
  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(std::move(m_driveWithController));
  m_intake.SetDefaultCommand(std::move(m_stopIntake));
  m_shooter.SetDefaultCommand(std::move(m_defaultShooter));
  m_stager.SetDefaultCommand(std::move(m_manualStager));
  m_climber.SetDefaultCommand(std::move(m_stopClimber));
  
  // Add auto name options
  m_autoChooser.SetDefaultOption("Speaker21", m_speaker21);
  m_autoChooser.AddOption("Speaker21Far", m_speaker21Far);
  m_autoChooser.AddOption("Speaker241", m_speaker241);
  m_autoChooser.AddOption("Speaker213", m_speaker213);
  m_autoChooser.AddOption("Speaker23", m_speaker23);
  m_autoChooser.AddOption("Speaker2", m_speaker2); 
  m_autoChooser.AddOption("Amp12", m_amp12);
  m_autoChooser.AddOption("Amp1", m_amp1);
  m_autoChooser.AddOption("Source3", m_source3);
  m_autoChooser.AddOption("Source32", m_source32);
  m_autoChooser.AddOption("ShootOne", m_shootOne);
  m_autoChooser.AddOption("SourceTravel", m_sourceTravel);
  frc::Shuffleboard::GetTab("Autonomous").Add(m_autoChooser);
}

void RobotContainer::ConfigureButtonBindings() {

  // Create new button bindings
  frc2::JoystickButton resetButton(&m_driveController, ControllerConstants::kResetGyroButtonIndex); 
  frc2::JoystickButton robotRelativeButton(&m_driveController, ControllerConstants::kRobotRelativeButtonIndex);
  frc2::JoystickButton fieldRelativeButton(&m_driveController, ControllerConstants::kFieldRelativeButtonIndex); 
  frc2::JoystickButton driveFacingGoalButton(&m_driveController, ControllerConstants::kDriveFacingGoalButtonIndex);
  frc2::JoystickButton smartShootWhileMovingButton(&m_driveController, ControllerConstants::kShootWhileMovingButtonIndex);
  frc2::JoystickButton visionIntakeButton(&m_driveController, ControllerConstants::kDriveToAmpButtonIndex);
  frc2::JoystickButton intakeButton(&m_operatorController, ControllerConstants::kIntakeButtonIndex); 
  frc2::JoystickButton outtakeButton(&m_operatorController, ControllerConstants::kOuttakeButtonIndex); 
  frc2::JoystickButton closeShootButton(&m_operatorController, ControllerConstants::kCloseShooterButton);
  frc2::JoystickButton farShooterButton(&m_operatorController, ControllerConstants::kFarShooterButton);
  frc2::JoystickButton ampShooterButton(&m_operatorController, ControllerConstants::kAmpShooterButton);
  frc2::JoystickButton NTEShooterButton(&m_operatorController, ControllerConstants::kNTEShooterButton);
  frc2::JoystickButton extendClimberButton(&m_driveController, ControllerConstants::kExtendShooterButtonIndex);
  frc2::JoystickButton retractClimberButton(&m_driveController, ControllerConstants::kRetractShooterButtonIndex);

  // Bind commands to button triggers
  resetButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.ZeroHeading();}, {}));
  robotRelativeButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.SetRobotRelative();}, {}));
  fieldRelativeButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.SetFieldRelative();}, {}));
  //driveFacingGoalButton.WhileTrue(DriveFacingGoal{&m_drive, &m_driveController}.ToPtr());
  driveFacingGoalButton.WhileTrue(DriveFacingGoal{&m_drive, &m_driveController}.ToPtr());
  intakeButton.WhileTrue(SmartIntake{&m_intake, &m_stager}.ToPtr());
  outtakeButton.WhileTrue(SmartOuttake{&m_intake, &m_stager}.ToPtr());
  extendClimberButton.WhileTrue(ExtendClimber{&m_climber}.ToPtr());
  retractClimberButton.WhileTrue(RetractClimber{&m_climber}.ToPtr());
  //NTEShooterButton.WhileTrue(ManualNteShooter{&m_shooter, &m_operatorController}.ToPtr());//SmartShooter{&m_shooter, &m_drive, &m_operatorController, &m_driveController}.ToPtr());

  smartShootWhileMovingButton.WhileTrue(SmartShootWhileMoving{&m_shooter, &m_drive, &m_operatorController, &m_driveController}.ToPtr());
  //visionIntakeButton.OnTrue(frc2::cmd::Parallel(m_drive.DriveToAmp(), SmartIntake{&m_intake, &m_stager}.ToPtr()));

  // Manual shooting buttons
  //closeShootButton.WhileTrue(ManualCloseShoot{&m_shooter}.ToPtr());
  closeShootButton.WhileTrue(SmartShooter{&m_shooter, &m_drive, &m_operatorController, &m_driveController}.ToPtr());
  farShooterButton.WhileTrue(ManualCloseShoot{&m_shooter}.ToPtr());
  ampShooterButton.WhileTrue(AmpShoot{&m_shooter, &m_stager}.ToPtr());

  /*
  NTEShooterButton.WhileTrue(frc2::cmd::Run([&] {
    m_shooter.SetShooterAngle((units::radian_t)1.0);
    m_shooter.SetIndividualShooterSpeed((units::revolutions_per_minute_t)150,(units::revolutions_per_minute_t)3700);
  }, {&m_shooter}));
  */
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // Builds and returns auto commands from pathplanner
  return PathPlannerAuto(m_autoChooser.GetSelected()).ToPtr();
}
