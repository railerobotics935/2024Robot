// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <iostream>
#include <vector>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>

#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/util/HolonomicPathFollowerConfig.h"
#include "pathplanner/lib/util/PIDConstants.h"
#include "pathplanner/lib/util/ReplanningConfig.h"

#include "Constants.h"
#include "utils/SwerveUtils.h"
#include "utils/MathUtils.h"

using namespace DriveConstants;

using namespace pathplanner;

DriveSubsystem::DriveSubsystem()
  : m_frontLeft{kFrontLeftDriveMotorPort,
                kFrontLeftTurningMotorPort,
                kFrontLeftDriveEncoderOffset},

    m_frontRight{
        kFrontRightDriveMotorPort,       
        kFrontRightTurningMotorPort,
        kFrontRightDriveEncoderOffset},
    
    m_backLeft{
        kBackLeftDriveMotorPort,       
        kBackLeftTurningMotorPort,
        kBackLeftDriveEncoderOffset},

    m_backRight{
        kBackRightDriveMotorPort,       
        kBackRightTurningMotorPort,  
        kBackRightDriveEncoderOffset},

    m_odometry{m_driveKinematics,
                -m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw),
                {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                m_backLeft.GetPosition(), m_backRight.GetPosition()},
                frc::Pose2d{(units::meter_t)3.0, (units::meter_t)3.0, -m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw)}},

    m_poseEstimator{m_driveKinematics,
                -m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw),
                {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                m_backLeft.GetPosition(), m_backRight.GetPosition()},
                frc::Pose2d{(units::meter_t)3.0, (units::meter_t)3.0, -m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw)},
                {0.05, 0.05, 0.05}, // Standard Deviation of the encoder position value
                {0.1, 0.1, 0.1}} // Standard Deviation of vision pose esitmation
{
  
// Configure the AutoBuilder last
AutoBuilder::configureHolonomic(
    [this](){ return GetPose(); }, // Robot pose supplier
    [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
    [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    [this](frc::ChassisSpeeds speeds){ DriveWithChassisSpeeds(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        PIDConstants(AutoConstants::kPTanslationController, AutoConstants::kITanslationController, AutoConstants::kDTanslationController), // Translation PID constants
        PIDConstants(AutoConstants::kPRotationController, AutoConstants::kIRotationController, AutoConstants::kDRotationController), // Rotation PID constants
        ModuleConstants::kModuleMaxLinearVelocity, // Max module speed, in m/s
        kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
        ReplanningConfig() // Default path replanning config. See the API for the options here
    ),
    // Supplier that determines if paths should be flipped to the other side of the field. This will maintain a global blue alliance origin.
    [this](){return InRedAlliance();},
    this // Reference to this subsystem to set requirements
  );

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("datatable");

  nte_fl_set_angle = nt_table->GetEntry("Swerve Drive/Front Left/Set Angle");
  nte_fr_set_angle = nt_table->GetEntry("Swerve Drive/Front Right/Set Angle");
  nte_bl_set_angle = nt_table->GetEntry("Swerve Drive/Back Left/Set Angle");
  nte_br_set_angle = nt_table->GetEntry("Swerve Drive/Back Right/Set Angle");
  nte_fl_set_speed = nt_table->GetEntry("Swerve Drive/Front Left/Set Speed");
  nte_fr_set_speed = nt_table->GetEntry("Swerve Drive/Front Right/Set Speed");
  nte_bl_set_speed = nt_table->GetEntry("Swerve Drive/Back Left/Set Speed");
  nte_br_set_speed = nt_table->GetEntry("Swerve Drive/Back Right/Set Speed");

  nte_fl_real_angle = nt_table->GetEntry("Swerve Drive/Front Left/Real Angle");
  nte_fr_real_angle = nt_table->GetEntry("Swerve Drive/Front Right/Real Angle");
  nte_bl_real_angle = nt_table->GetEntry("Swerve Drive/Back Left/Real Angle");
  nte_br_real_angle = nt_table->GetEntry("Swerve Drive/Back Right/Real Angle");
  nte_fl_real_speed = nt_table->GetEntry("Swerve Drive/Front Left/Real Speed");
  nte_fr_real_speed = nt_table->GetEntry("Swerve Drive/Front Right/Real Speed");
  nte_bl_real_speed = nt_table->GetEntry("Swerve Drive/Back Left/Real Speed");
  nte_br_real_speed = nt_table->GetEntry("Swerve Drive/Back Right/Real Speed");

  nte_fl_raw_encoder_voltage = nt_table->GetEntry("Swerve Drive/Front Left/Encoder Voltage");
  nte_fr_raw_encoder_voltage = nt_table->GetEntry("Swerve Drive/Front Right/Encoder Voltage");
  nte_bl_raw_encoder_voltage = nt_table->GetEntry("Swerve Drive/Back Left/Encoder Voltage");
  nte_br_raw_encoder_voltage = nt_table->GetEntry("Swerve Drive/Back Right/Encoder Voltage");


  nte_gyro_angle = nt_table->GetEntry("Swerve Drive/Gyro Angle");
  nte_robot_x = nt_table->GetEntry("Swerve Drive/Robot X");
  nte_robot_y = nt_table->GetEntry("Swerve Drive/Robot Y");

  // Send Field to shuffleboard
  frc::Shuffleboard::GetTab("Field").Add(m_field);

}

// Returns true is the allience selected is red
bool DriveSubsystem::InRedAlliance() {
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    return false;
  else
    return true;
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(-m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
                    m_backLeft.GetPosition(), m_backRight.GetPosition()});

  // set odometry relative to the apriltag
  EstimatePoseWithApriltag();
                    
  nte_fl_real_angle.SetDouble((double)m_frontLeft.GetState().angle.Radians());
  nte_fr_real_angle.SetDouble((double)m_frontRight.GetState().angle.Radians());
  nte_bl_real_angle.SetDouble((double)m_backLeft.GetState().angle.Radians());
  nte_br_real_angle.SetDouble((double)m_backRight.GetState().angle.Radians());
  nte_fl_real_speed.SetDouble((double)m_frontLeft.GetState().speed);
  nte_fr_real_speed.SetDouble((double)m_frontRight.GetState().speed);
  nte_bl_real_speed.SetDouble((double)m_backLeft.GetState().speed);
  nte_br_real_speed.SetDouble((double)m_backRight.GetState().speed);

  nte_gyro_angle.SetDouble((double)m_odometry.GetPose().Rotation().Radians());
  nte_robot_x.SetDouble((double)m_odometry.GetPose().X());
  nte_robot_y.SetDouble((double)m_odometry.GetPose().Y());

  //nte_fl_raw_encoder_voltage.SetDouble(m_frontLeft.GetEncoderVoltage());
  //nte_fr_raw_encoder_voltage.SetDouble(m_frontRight.GetEncoderVoltage());
  //nte_bl_raw_encoder_voltage.SetDouble(m_backLeft.GetEncoderVoltage());
  //nte_br_raw_encoder_voltage.SetDouble(m_backRight.GetEncoderVoltage());

  // Set robot position to shuffleboard field
  m_field.SetRobotPose(m_poseEstimator.GetEstimatedPosition());

  // Update robot distance from goal
  nte_robot_distance_to_goal.SetDouble((double)MathUtils::RobotDistanceToGoal(m_poseEstimator.GetEstimatedPosition()));
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool rateLimit) {
                             
  double xSpeedCommanded;
  double ySpeedCommanded;

  if (rateLimit) {
    // Convert XY to polar for rate limiting
    double inputTranslationDir = atan2(ySpeed.value(), xSpeed.value());
    double inputTranslationMag =
        sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));

    // Calculate the direction slew rate based on an estimate of the lateral
    // acceleration
    double directionSlewRate;
    if (m_currentTranslationMag != 0.0) {
      directionSlewRate =
          abs(DriveConstants::kDirectionSlewRate / m_currentTranslationMag);
    } else {
      directionSlewRate = 500.0;  // some high number that means the slew rate
                                  // is effectively instantaneous
    }

    double currentTime = wpi::Now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDif = SwerveUtils::AngleDifference(inputTranslationDir,
                                                   m_currentTranslationDir);
    if (angleDif < 0.45 * std::numbers::pi) {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
    } 
    else if (angleDif > 0.85 * std::numbers::pi) {
      if (m_currentTranslationMag > 1e-4) {  // some small number to avoid floating-point errors with equality checking
        // keep currentTranslationDir unchanged
        m_currentTranslationMag = m_magLimiter.Calculate(0.0);
      } 
      else {
        m_currentTranslationDir =
            SwerveUtils::WrapAngle(m_currentTranslationDir + std::numbers::pi);
        m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
      }
    } 
    else {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(0.0);
    }
    m_prevTime = currentTime;

    xSpeedCommanded = m_currentTranslationMag * cos(m_currentTranslationDir);
    ySpeedCommanded = m_currentTranslationMag * sin(m_currentTranslationDir);
    m_currentRotation = m_rotLimiter.Calculate(rot.value());

  } 
  else {
    xSpeedCommanded = xSpeed.value();
    ySpeedCommanded = ySpeed.value();
    m_currentRotation = rot.value();
  }

  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeedCommanded * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeedCommanded * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      m_currentRotation * DriveConstants::kMaxAngularSpeed;

  auto states = m_driveKinematics.ToSwerveModuleStates(
      m_fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{
                    -m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  m_driveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);

  // Network table entries
  nte_fl_set_angle.SetDouble((double)fl.angle.Radians());
  nte_fr_set_angle.SetDouble((double)fr.angle.Radians());
  nte_bl_set_angle.SetDouble((double)bl.angle.Radians());
  nte_br_set_angle.SetDouble((double)br.angle.Radians());
  nte_fl_set_speed.SetDouble((double)fl.speed);
  nte_fr_set_speed.SetDouble((double)fr.speed);
  nte_bl_set_speed.SetDouble((double)bl.speed);
  nte_br_set_speed.SetDouble((double)br.speed);
  
}

void DriveSubsystem::DriveFacingGoal(units::meters_per_second_t xSpeed,
                                      units::meters_per_second_t ySpeed, 
                                      frc::Rotation2d rot, 
                                      bool rateLimit) {

                                    
 }

void DriveSubsystem::DriveWithChassisSpeeds(frc::ChassisSpeeds speeds) {
  auto states = m_driveKinematics.ToSwerveModuleStates(speeds);

  m_driveKinematics.DesaturateWheelSpeeds(&states, ModuleConstants::kModuleMaxLinearVelocity);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  m_driveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         ModuleConstants::kModuleMaxLinearVelocity);
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
  return -m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw);
}

void DriveSubsystem::ZeroHeading() {
  m_gyro.Reset();
}

void DriveSubsystem::SetRobotRelative() {
  m_fieldRelative = false;
}

void DriveSubsystem::SetFieldRelative() {
  m_fieldRelative = true;
}

double DriveSubsystem::GetTurnRate() {
  return (double)-m_gyro.GetRate(frc::ADIS16470_IMU::kYaw);
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_poseEstimator.GetEstimatedPosition();
}

frc::ChassisSpeeds DriveSubsystem::GetRobotRelativeSpeeds()
{
  return m_driveKinematics.ToChassisSpeeds(m_frontLeft.GetState(), 
                                           m_frontRight.GetState(),
                                           m_backLeft.GetState(),
                                           m_backRight.GetState());
}

// This needs to be changed
frc::ChassisSpeeds DriveSubsystem::GetFieldRelativeSpeeds()
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

void DriveSubsystem::EstimatePoseWithApriltag() {
  
  m_poseEstimator.Update(-m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw), 
                        {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backLeft.GetPosition(), m_backRight.GetPosition()});
                
  // Iterate through each tag, adding it to the pose estimator if it is tracked
  for (int tag = 1; tag <= 16; tag++ ) { // Check each tag for each camera
    if (m_frontCameraSensor.TagIsTracked(tag) && m_frontCameraSensor.GetTimestamp(tag) > (units::second_t)0.0)
      m_poseEstimator.AddVisionMeasurement(m_frontCameraSensor.GetFieldRelativePose(tag).ToPose2d(), m_frontCameraSensor.GetTimestamp(tag));
    if (m_backCameraSensor.TagIsTracked(tag) && m_backCameraSensor.GetTimestamp(tag) > (units::second_t)0.0)
      m_poseEstimator.AddVisionMeasurement(m_backCameraSensor.GetFieldRelativePose(tag).ToPose2d(), m_backCameraSensor.GetTimestamp(tag));
  }
} 