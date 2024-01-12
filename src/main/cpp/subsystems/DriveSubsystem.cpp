// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/util/HolonomicPathFollowerConfig.h"
#include "pathplanner/lib/util/PIDConstants.h"
#include "pathplanner/lib/util/ReplanningConfig.h"

#include <iostream>
#include "Constants.h"

using namespace DriveConstants;

using namespace pathplanner;

DriveSubsystem::DriveSubsystem()
  : m_frontLeft{kFrontLeftDriveMotorPort,
                kFrontLeftTurningMotorPort,
                kFrontLeftTurningEncoderPort,
                kFrontLeftDriveEncoderOffset},

    m_frontRight{
        kFrontRightDriveMotorPort,       
        kFrontRightTurningMotorPort,
        kFrontRightTurningEncoderPort,
        kFrontRightDriveEncoderOffset},
    
    m_backLeft{
        kBackLeftDriveMotorPort,       
        kBackLeftTurningMotorPort,
        kBackLeftTurningEncoderPort,
        kBackLeftDriveEncoderOffset},

    m_backRight{
        kBackRightDriveMotorPort,       
        kBackRightTurningMotorPort,  
        kBackRightTurningEncoderPort,
        kBackRightDriveEncoderOffset},

    m_odometry{m_driveKinematics,
                m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw),
                {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                m_backLeft.GetPosition(), m_backRight.GetPosition()},
                frc::Pose2d{}} 
{
  
// TODO: Create Allience side supplyer for autobuilder
// Configure the AutoBuilder last
AutoBuilder::configureHolonomic(
    [this](){ return GetPose(); }, // Robot pose supplier
    [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
    [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    [this](frc::ChassisSpeeds speeds){ DriveWithChassisSpeeds(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
        4.5_mps, // Max module speed, in m/s
        0.4_m, // Drive base radius in meters. Distance from robot center to furthest module.
        ReplanningConfig() // Default path replanning config. See the API for the options here
    ),
    [this](){return false;}, // Supplier that determines if paths should be flipped to the other side of the field. This will maintain a global blue alliance origin.
    this // Reference to this subsystem to set requirements
);

}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
                    m_backLeft.GetPosition(), m_backRight.GetPosition()});
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  auto states = m_driveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw))
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_driveKinematics.DesaturateWheelSpeeds(&states, ModuleConstants::kMaxLinearVelocity);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void DriveSubsystem::DriveWithChassisSpeeds(frc::ChassisSpeeds speeds) {
  auto states = m_driveKinematics.ToSwerveModuleStates(speeds);

  m_driveKinematics.DesaturateWheelSpeeds(&states, ModuleConstants::kMaxLinearVelocity);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  m_driveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         ModuleConstants::kMaxLinearVelocity);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_backLeft.SetDesiredState(desiredStates[2]);
  m_backRight.SetDesiredState(desiredStates[3]);
}

// Method to put the Robot in Park
void DriveSubsystem::Park()
{
  frc::SwerveModuleState fl;
  frc::SwerveModuleState fr;
  frc::SwerveModuleState bl;
  frc::SwerveModuleState br;

  fl.angle = frc::Rotation2d (units::radian_t(std::numbers::pi / 4));
  fr.angle = frc::Rotation2d (units::radian_t(-std::numbers::pi / 4));
  bl.angle = frc::Rotation2d (units::radian_t(-std::numbers::pi / 4));
  br.angle = frc::Rotation2d (units::radian_t(std::numbers::pi / 4));

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw);
}

void DriveSubsystem::ZeroHeading() {
  m_gyro.Reset();
}

double DriveSubsystem::GetTurnRate() {
  return (double)-m_gyro.GetRate(frc::ADIS16470_IMU::kYaw);
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

frc::ChassisSpeeds DriveSubsystem::GetRobotRelativeSpeeds()
{
  return m_driveKinematics.ToChassisSpeeds(m_frontLeft.GetState(), 
                                           m_frontRight.GetState(),
                                           m_backLeft.GetState(),
                                           m_backRight.GetState());
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      pose);
}
