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

#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;
using namespace pathplanner;

RobotContainer::RobotContainer() {
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureButtonBindings();

    // Set up default drive command
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
            const auto ySpeed = -m_ySpeedLimiter.Calculate(frc::ApplyDeadband(m_driveController.GetRawAxis(ControllerConstants::kDriveLeftYIndex), 0.15)) * kRobotMaxLinearVelocity;
            const auto xSpeed = -m_xSpeedLimiter.Calculate(frc::ApplyDeadband(m_driveController.GetRawAxis(ControllerConstants::kDriveLeftXIndex), 0.15)) * kRobotMaxLinearVelocity;
            const auto rot = -m_rotLimiter.Calculate(frc::ApplyDeadband(m_driveController.GetRawAxis(ControllerConstants::kDriveRightXIndex), 0.15)) * kRobotMaxAngularVelocity;
            m_drive.Drive(
                units::meters_per_second_t{ySpeed},
                units::meters_per_second_t{xSpeed},
                units::radians_per_second_t{rot}, 
                m_driveController.GetRawButton(ControllerConstants::kFieldRelativeSwitchIndex));
        },
        {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
    
    frc2::JoystickButton resetButton(&m_driveController, ControllerConstants::kResetGyroButtonIndex); // Creates a new JoystickButton object for the "reset" button on Drive Controller    
    frc2::JoystickButton slowSwitch(&m_driveController, ControllerConstants::kSlowStateSwitchIndex); // Creates a new JoystickButton object for the slow switch on Drive Controller    
    frc2::JoystickButton parkSwitch(&m_driveController, ControllerConstants::kParkSwitchIndex); // Creates a new JoystickButton object for the brake switch on Drive Controller    

    // I don't exactly know why this works, but the documentation for command based c++ is kinda bad 
    resetButton.OnTrue(frc2::cmd::Run([&] {m_drive.ZeroHeading();}, {&m_drive}));
    slowSwitch.WhileTrue(frc2::cmd::Run([&] {            
            const auto ySpeed = -m_ySpeedLimiter.Calculate(frc::ApplyDeadband(m_driveController.GetRawAxis(ControllerConstants::kDriveLeftYIndex), 0.15)) * kRobotMaxLinearVelocity;
            const auto xSpeed = -m_xSpeedLimiter.Calculate(frc::ApplyDeadband(m_driveController.GetRawAxis(ControllerConstants::kDriveLeftXIndex), 0.15)) * kRobotMaxLinearVelocity;
            const auto rot = -m_rotLimiter.Calculate(frc::ApplyDeadband(m_driveController.GetRawAxis(ControllerConstants::kDriveRightXIndex), 0.15)) * kRobotMaxAngularVelocity;
            m_drive.Drive(
                units::meters_per_second_t{ySpeed * 0.25},
                units::meters_per_second_t{xSpeed * 0.25},
                units::radians_per_second_t{rot}, 
                m_driveController.GetRawButton(ControllerConstants::kFieldRelativeSwitchIndex));
        }, 
        {&m_drive}));

    parkSwitch.WhileTrue(frc2::cmd::Run([&] {m_drive.Park();}, {&m_drive}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {

// builds and return auto command from pathplanner
return PathPlannerAuto("Example Auto").ToPtr();


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
