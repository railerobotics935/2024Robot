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

#include "pathplanner/lib/commands/PathPlannerAuto.h"
#include "pathplanner/lib/auto/NamedCommands.h"

#include "commands/drive/DriveWithController.h"
#include "commands/drive/DriveFacingGoal.h"
#include "commands/drive/SlowDrive.h"

#include "commands/intake/SimpleIntake.h"
#include "commands/intake/SmartIntake.h"
#include "commands/intake/SmartOuttake.h"

#include "commands/shooter/ManualNteShooter.h"
#include "commands/shooter/SmartShooting.h"
#include "commands/shooter/ManualCloseShoot.h"
#include "commands/shooter/ManualFarShoot.h"

#include "subsystems/DriveSubsystem.h"
#include "Constants.h"

using namespace DriveConstants;
using namespace pathplanner;

RobotContainer::RobotContainer() : m_shooter{ShooterConstants::kPitchOffset} {
  m_revPDH.SetSwitchableChannel(false); //-------------------------------------------------------------------------------------

  // Initialize all of your commands and subsystems here
  // Configuring command bindings for pathplanner
  // TODO: Create custom commands for all of these
  NamedCommands::registerCommand("SmartIntake", SmartIntake{&m_intake, &m_stager}.ToPtr());
  NamedCommands::registerCommand("SetShooterSpeeds", ManualCloseShoot{&m_shooter}.ToPtr());
  NamedCommands::registerCommand("SetFarShooterSpeeds", ManualFarShoot{&m_shooter}.ToPtr());

  NamedCommands::registerCommand("StageForShooting", frc2::cmd::RunOnce([&] {
    m_stager.SetMotorPower(1.0);
  }, {&m_stager}));

  NamedCommands::registerCommand("EndShooting", frc2::cmd::RunOnce([&] {
    m_stager.SetMotorPower(0.0);
  }, {&m_stager}));

  NamedCommands::registerCommand("StopEverything", frc2::cmd::RunOnce([&] {
    m_stager.SetMotorPower(0.0);
    m_shooter.SetShooterMotorPower(0.0);
  }, {&m_shooter, &m_stager}));
  
  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(DriveWithController{&m_drive, &m_driveController}.ToPtr());
  
  m_intake.SetDefaultCommand(frc2::RunCommand([this] {m_intake.SetMotorPower(0.0);}, {&m_intake}));

  m_shooter.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_shooter.SetShooterMotorPower(-frc::ApplyDeadband(m_operatorController.GetRawAxis(ControllerConstants::kOperatorLeftYIndex), 0.05));
      m_shooter.SetShooterAngle((units::radian_t)0.7);
    }, {&m_shooter}
  ));

  m_stager.SetDefaultCommand(frc2::RunCommand(
    [this] {
      if (frc::ApplyDeadband(m_operatorController.GetRawAxis(ControllerConstants::kStagerIntakeTrigger), 0.05) != 0)
        m_stager.SetMotorPower(-frc::ApplyDeadband(m_operatorController.GetRawAxis(ControllerConstants::kStagerIntakeTrigger), 0.05));
      else
        m_stager.SetMotorPower(frc::ApplyDeadband(m_operatorController.GetRawAxis(ControllerConstants::kStagerOuttakeTrigger), 0.05));
    }, {&m_stager}
  ));

  m_climber.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_climber.SetClimberPower(0.0);
    }, {&m_climber}
  ));

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
  //frc2::JoystickButton slowButton(&m_driveController, ControllerConstants::kSlowStateButtonIndex); 
  //frc2::JoystickButton driveFacingGoalButton(&m_driveController, ControllerConstants::kDriveFacingGoalButtonIndex);
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
  //driveFacingGoalButton.ToggleOnTrue(DriveFacingGoal{&m_drive, &m_driveController}.ToPtr());
  //slowButton.ToggleOnTrue(SlowDrive{&m_drive, &m_driveController}.ToPtr());
  intakeButton.WhileTrue(SmartIntake{&m_intake, &m_stager}.ToPtr());
  outtakeButton.WhileTrue(SmartOuttake{&m_intake, &m_stager}.ToPtr());
  //NTEShooterButton.WhileTrue(ManualNteShooter{&m_shooter, &m_operatorController}.ToPtr());//SmartShooter{&m_shooter, &m_drive, &m_operatorController, &m_driveController}.ToPtr());

  // Manual shooting buttons
  closeShootButton.WhileTrue(ManualCloseShoot{&m_shooter}.ToPtr());
  farShooterButton.WhileTrue(ManualFarShoot{&m_shooter}.ToPtr());

  /*
  ampShooterButton.WhileTrue(frc2::cmd::Run([&] {
    m_shooter.SetShooterAngle((units::radian_t)1.1);
    m_shooter.SetIndividualShooterSpeed((units::revolutions_per_minute_t)100,(units::revolutions_per_minute_t)4500);
  }, {&m_shooter}));
  
  NTEShooterButton.WhileTrue(frc2::cmd::Run([&] {
    m_shooter.SetShooterAngle((units::radian_t)1.0);
    m_shooter.SetIndividualShooterSpeed((units::revolutions_per_minute_t)150,(units::revolutions_per_minute_t)3700);
  }, {&m_shooter}));
  */

  extendClimberButton.WhileTrue(frc2::cmd::Run([&] {
    m_climber.SetClimberPower(-1.0);
  }, {&m_climber}));

  retractClimberButton.WhileTrue(frc2::cmd::Run([&] {
    m_climber.SetClimberPower(1.0);
  }, {&m_climber}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // Builds and returns auto commands from pathplanner
  return PathPlannerAuto(m_autoChooser.GetSelected()).ToPtr();
}
