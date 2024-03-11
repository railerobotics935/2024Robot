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
#include "commands/EstimatePose.h"


#include "subsystems/DriveSubsystem.h"
#include "Constants.h"

using namespace DriveConstants;
using namespace pathplanner;

RobotContainer::RobotContainer() : m_shooter{ShooterConstants::kPitchOffset} {

  // Initialize all of your commands and subsystems here
  // Configuring command bindings for pathplanner
  NamedCommands::registerCommand("SimpleIntake", SimpleIntake{&m_intake}.ToPtr());
  
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

  // Add auto name options
  m_autoChooser.SetDefaultOption("Trapezoid Test", m_trapezoidTest);
  m_autoChooser.AddOption("Forward 1m", m_forward1m);
  m_autoChooser.AddOption("Left 1m", m_left1m);
  m_autoChooser.AddOption("Test Intake", m_testIntake);

  frc::Shuffleboard::GetTab("Autonomous").Add(m_autoChooser);
}

void RobotContainer::ConfigureButtonBindings() {
  
  // Create new button bindings
  frc2::JoystickButton resetButton(&m_driveController, ControllerConstants::kResetGyroButtonIndex); 
  frc2::JoystickButton robotRelativeButton(&m_driveController, ControllerConstants::kRobotRelativeButtonIndex);
  frc2::JoystickButton fieldRelativeButton(&m_driveController, ControllerConstants::kFieldRelativeButtonIndex); 
  frc2::JoystickButton slowButton(&m_driveController, ControllerConstants::kSlowStateButtonIndex); 
  frc2::JoystickButton driveFacingGoalButton(&m_driveController, ControllerConstants::kDriveFacingGoalButtonIndex);
  frc2::JoystickButton intakeButton(&m_operatorController, ControllerConstants::kIntakeButtonIndex); 
  frc2::JoystickButton outtakeButton(&m_operatorController, ControllerConstants::kOuttakeButtonIndex); 
  frc2::JoystickButton closeShootButton(&m_operatorController, ControllerConstants::kCloseShooterButton);
  frc2::JoystickButton farShooterButton(&m_operatorController, ControllerConstants::kFarShooterButton);
  frc2::JoystickButton ampShooterButton(&m_operatorController, ControllerConstants::kAmpShooterButton);
  frc2::JoystickButton NTEShooterButton(&m_operatorController, ControllerConstants::kNTEShooterButton);

  // Bind commands to button triggers
  resetButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.ZeroHeading();}, {}));
  robotRelativeButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.SetRobotRelative();}, {}));
  fieldRelativeButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.SetFieldRelative();}, {}));
  driveFacingGoalButton.ToggleOnTrue(DriveFacingGoal{&m_drive, &m_driveController}.ToPtr());
  slowButton.ToggleOnTrue(SlowDrive{&m_drive, &m_driveController}.ToPtr());
  intakeButton.WhileTrue(SmartIntake{&m_intake, &m_stager}.ToPtr());
  outtakeButton.WhileTrue(SmartOuttake{&m_intake, &m_stager}.ToPtr());
  NTEShooterButton.WhileTrue(SmartShooter{&m_shooter, &m_drive, &m_operatorController, &m_driveController}.ToPtr());

  // Manual shooting buttons
  closeShootButton.WhileTrue(frc2::cmd::Run([&] {
    m_shooter.SetShooterAngle((units::radian_t)1.00);
    m_shooter.SetShooterSpeed((units::revolutions_per_minute_t)8500);
  }, {&m_shooter}));

  farShooterButton.WhileTrue(frc2::cmd::Run([&] {
    m_shooter.SetShooterAngle((units::radian_t)0.8);
    m_shooter.SetShooterSpeed((units::revolutions_per_minute_t)8500);
  }, {&m_shooter}));

  ampShooterButton.WhileTrue(frc2::cmd::Run([&] {
    m_shooter.SetShooterAngle((units::radian_t)1.05);
    m_shooter.SetIndivualShooterSpeed((units::revolutions_per_minute_t)200,(units::revolutions_per_minute_t)3300);
  }, {&m_shooter}));
}

// Currently Not in use. Estimate pose currently runs in the periodit cycle of drivetrain
frc2::CommandPtr RobotContainer::GetRobotCommand() {
  return EstimatePose{&m_drive}.ToPtr();
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // Builds and returns auto commands from pathplanner
  return PathPlannerAuto(m_autoChooser.GetSelected()).ToPtr();
}
