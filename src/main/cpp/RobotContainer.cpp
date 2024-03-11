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
#include "commands/intake/SimpleIntake.h"
#include "commands/drive/SlowDrive.h"
#include "commands/shooter/ManualNteShooter.h"
#include "commands/intake/SmartIntake.h"
#include "commands/EstimatePose.h"
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

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
      m_shooter.SetShooterAngle((units::radian_t)0.6);
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
  frc2::JoystickButton closeShootButton(&m_operatorController, ControllerConstants::kCloseShooterButton);
  frc2::JoystickButton farShooterButton(&m_operatorController, ControllerConstants::kFarShooterButton);
  frc2::JoystickButton ampShooterButton(&m_operatorController, ControllerConstants::kAmpShooterButton);
  frc2::JoystickButton NTEShooterButton(&m_operatorController, ControllerConstants::kNTEShooterButton);

  // Bind commands to button triggers
  resetButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.ZeroHeading();}, {}));
  robotRelativeButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.SetRobotRelative();}, {}));
  fieldRelativeButton.OnTrue(frc2::cmd::RunOnce([&] {m_drive.SetFieldRelative();}, {}));
  driveFacingGoalButton.WhileTrue(DriveFacingGoal{&m_drive, &m_driveController}.ToPtr());
  slowButton.ToggleOnTrue(SlowDrive{&m_drive, &m_driveController}.ToPtr());
  intakeButton.WhileTrue(SmartIntake{&m_intake, &m_stager}.ToPtr());
  NTEShooterButton.WhileTrue(ManualNteShooter{&m_shooter, &m_operatorController}.ToPtr());

  // Manual shooting buttons
  closeShootButton.WhileTrue(frc2::cmd::Run([&] {
    m_shooter.SetShooterAngle((units::radian_t)1.00);
    m_shooter.SetShooterSpeed((units::revolutions_per_minute_t)9000);
  }, {&m_shooter}));

  farShooterButton.WhileTrue(frc2::cmd::Run([&] {
    m_shooter.SetShooterAngle((units::radian_t)0.8);
    m_shooter.SetShooterSpeed((units::revolutions_per_minute_t)8000);
  }, {&m_shooter}));

  ampShooterButton.WhileTrue(frc2::cmd::Run([&] {
    m_shooter.SetShooterAngle((units::radian_t)1.05);
    m_shooter.SetIndivualShooterSpeed((units::revolutions_per_minute_t)200,(units::revolutions_per_minute_t)3300);
  }, {&m_shooter}));
}

  frc2::CommandPtr RobotContainer::GetRobotCommand() {
    return EstimatePose{&m_drive}.ToPtr();
  }

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  
  // Reset Odometry and only start with where path planner thinks the robot is
  // This will go away once we use apriltags to update our robot pose during disabled
  //m_drive.ResetOdometry(frc::Pose2d((units::meter_t)0.0, (units::meter_t)0.0, (units::radian_t)0.0));

  // Builds and returns auto commands from pathplanner
  return PathPlannerAuto(m_autoChooser.GetSelected()).ToPtr();

// Basic wpilib trajectory follow
#if 0
  // Set up config for trajectory
  frc::TrajectoryConfig config{AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration};

    frc::ProfiledPIDController<units::radians> thetaController{
       5, 0, 0,
        AutoConstants::kThetaControllerConstraints};

 // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d{1_m, -1_m}, frc::Translation2d{2_m, 1_m}, frc::Translation2d{3_m, 0_m}},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{0_m, 0_m, 180_deg},
      // Pass the config
      config);

frc2::SwerveControllerCommand<4> swerveControllerCommand(
    exampleTrajectory, [this]() { return m_drive.GetPose(); },

    m_drive.m_driveKinematics,

    frc::PIDController{AutoConstants::kPXController, 0, 0},
    frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

    [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

    {&m_drive});

// Reset odometry to the starting pose of the trajectory.
m_drive.ResetOdometry(exampleTrajectory.InitialPose());

// no auto
return new frc2::SequentialCommandGroup(
    std::move(swerveControllerCommand),
    frc2::InstantCommand(
        [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {}));
    
#endif
}
