// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <iostream>
#include <vector>
#include <cmath>
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
                {0.05, 0.05, 0.001}, // Standard Deviation of the encoder position value
                {0.2, 0.2, 0.05}} // Standard Deviation of vision pose esitmation
{
  
// Configure the AutoBuilder last
AutoBuilder::configureHolonomic(
    [this](){ return GetOdometryPose(); }, // Robot pose supplier
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
    // Supplier that determines if paths should be flipped to the other side of the field. This will helps keep the cordinate system on the blue side
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

  nte_kp = nt_table->GetEntry("SwerveDrive/PID/KP");
  nte_ki = nt_table->GetEntry("SwerveDrive/PID/KI");
  nte_kd = nt_table->GetEntry("SwerveDrive/PID/KD"); 

  nte_debugTimeForPoseEstimation = nt_table->GetEntry("Debug Values/Pose Estimation");
  nte_debugTimeForAddVistionData = nt_table->GetEntry("Debug Values/Add Vision Data");  
  nte_numberOfTagsAdded = nt_table->GetEntry("Debug Values/Number Of Tags Processed");

  //nte_kp.SetDouble(2.5);
  //nte_ki.SetDouble(0.002);
  //nte_kd.SetDouble(0.05);

  nte_robot_distance_to_goal = nt_table->GetEntry("Pose Estimation/Distance to Goal");
  
  // Send Field to shuffleboard
  frc::Shuffleboard::GetTab("Field").Add(m_field);

  m_robotAngleController.EnableContinuousInput(0, (std::numbers::pi * 2));

  m_timer.Restart();
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

  m_poseEstimator.Update(-m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw), 
                      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backLeft.GetPosition(), m_backRight.GetPosition()});

  // set odometry relative to the apriltag
  if (GetLinearRobotSpeed() < 1.0 && GetTurnRate() < 20.0)
    EstimatePoseWithApriltag();
  
  //UpdateNTE();

  m_field.SetRobotPose(m_poseEstimator.GetEstimatedPosition());

  //m_robotAngleController.SetP(nte_kp.GetDouble(4.5));
  //m_robotAngleController.SetI(nte_ki.GetDouble(0.002));
  //m_robotAngleController.SetD(nte_kd.GetDouble(0.05));
}

void DriveSubsystem::UpdateNTE() {
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
  nte_robot_distance_to_goal.SetDouble((double)RobotDistanceToGoal(m_poseEstimator.GetEstimatedPosition()));

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
  if (!m_fieldRelative) {
    xSpeedCommanded = -xSpeedCommanded; 
    ySpeedCommanded = -ySpeedCommanded; 
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
  //nte_fl_set_angle.SetDouble((double)fl.angle.Radians());
  //nte_fr_set_angle.SetDouble((double)fr.angle.Radians());
  //nte_bl_set_angle.SetDouble((double)bl.angle.Radians());
  //nte_br_set_angle.SetDouble((double)br.angle.Radians());
  //nte_fl_set_speed.SetDouble((double)fl.speed);
  //nte_fr_set_speed.SetDouble((double)fr.speed);
  //nte_bl_set_speed.SetDouble((double)bl.speed);
  //nte_br_set_speed.SetDouble((double)br.speed);
  
}

void DriveSubsystem::DriveFacingGoal(units::meters_per_second_t xSpeed,
                                      units::meters_per_second_t ySpeed, 
                                      frc::Rotation2d rotation, 
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
    m_currentRotation = m_rotLimiter.Calculate(m_robotAngleController.Calculate(
      (double)GetPose().Rotation().Radians(), (double)rotation.Radians()));
      //(double)(GetHeading() * std::numbers::pi / 180.0), (double)rotation.Radians()));

  } 
  else {
    xSpeedCommanded = xSpeed.value();
    ySpeedCommanded = ySpeed.value();
    m_currentRotation = m_robotAngleController.Calculate((double)GetPose().Rotation().Radians(), (double)rotation.Radians());
  }
  if (!m_fieldRelative) {
    xSpeedCommanded = -xSpeedCommanded; 
    ySpeedCommanded = -ySpeedCommanded; 
  }

  // Put limits so we don't go over the rotation speed limit
  if (m_currentRotation > 1.0)
    m_currentRotation = 1.0;
  if (m_currentRotation < -1.0)
    m_currentRotation = -1.0;

  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeedCommanded * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeedCommanded * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      m_currentRotation * (units::radians_per_second_t)(30.0 / (2 * std::numbers::pi)); // could limit this to go a slower speed

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
  //nte_fl_set_angle.SetDouble((double)fl.angle.Radians());
  //nte_fr_set_angle.SetDouble((double)fr.angle.Radians());
  //nte_bl_set_angle.SetDouble((double)bl.angle.Radians());
  //nte_br_set_angle.SetDouble((double)br.angle.Radians());
  //nte_fl_set_speed.SetDouble((double)fl.speed);
  //nte_fr_set_speed.SetDouble((double)fr.speed);
  //nte_bl_set_speed.SetDouble((double)bl.speed);
  //nte_br_set_speed.SetDouble((double)br.speed);
 }

bool DriveSubsystem::AtAngleSetpoint() {
  return m_robotAngleController.AtSetpoint(); 
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

units::degree_t DriveSubsystem::GetHeading() const {
  return -m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw);
}

double DriveSubsystem::GetLinearRobotSpeed() {
  // get magnitude of robot speed vector
  return sqrt(pow((double)GetRobotRelativeSpeeds().vx, 2) + pow((double)GetRobotRelativeSpeeds().vy, 2));
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

frc::Pose2d DriveSubsystem::GetOdometryPose() {
  return m_odometry.GetPose();
}

frc::ChassisSpeeds DriveSubsystem::GetRobotRelativeSpeeds()
{
  return m_driveKinematics.ToChassisSpeeds(m_frontLeft.GetState(), 
                                           m_frontRight.GetState(),
                                           m_backLeft.GetState(),
                                           m_backRight.GetState());
}

frc::ChassisSpeeds DriveSubsystem::GetFieldRelativeSpeeds()
{
  return frc::ChassisSpeeds{(units::meters_per_second_t)((GetRobotRelativeSpeeds().vx() * std::cos((double)GetPose().Rotation().Radians())) + // Vx componet from the robot X velocity
                                                         (GetRobotRelativeSpeeds().vy() * std::cos((double)GetPose().Rotation().Radians() + (std::numbers::pi/2)))), // Vx componet from the robot Y velocity

                            (units::meters_per_second_t)((GetRobotRelativeSpeeds().vy() * std::sin((double)GetPose().Rotation().Radians() + (std::numbers::pi/2))) + // Vx componet from the robot X velocity
                                                         (GetRobotRelativeSpeeds().vx() * std::sin((double)GetPose().Rotation().Radians()))), // Vx componet from the robot Y velocity

                            (units::radians_per_second_t)GetRobotRelativeSpeeds().omega()}; // Rotational Velocity stays the same
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      pose);
  m_poseEstimator.ResetPosition(      
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      pose);
}

void DriveSubsystem::EstimatePoseWithApriltag() {
#ifdef DEBUGPOSEESTIMATION
  double startEstiamtionTime = (double)m_timer.GetFPGATimestamp();
  int numberOfValidTags = 0;
#endif
  // Iterate through each tag, adding it to the pose estimator if it is tracked
  for (int tag = 1; tag <= 16; tag++ ) { // Check each tag for each camera
  //int tag = 7;
    // Front Camera
    if (m_frontCameraSensor.TagIsTracked(tag) && m_frontCameraSensor.GetTimestamp(tag) > (units::second_t)0.0){
      #ifdef DEBUGPOSEESTIMATION
      double startEstiamtionTime2 = (double)m_timer.GetFPGATimestamp();
      #endif

      m_poseEstimator.AddVisionMeasurement(m_frontCameraSensor.GetFieldRelativePose(tag).ToPose2d(), m_frontCameraSensor.GetTimestamp(tag), m_frontCameraSensor.GetStandardDeviations(tag));
      
      #ifdef DEBUGPOSEESTIMATION
      nte_debugTimeForAddVistionData.SetDouble((double)m_timer.GetFPGATimestamp() - startEstiamtionTime2);
      numberOfValidTags++;
      #endif
    }

    // Back Left Camera
    if (m_backLeftCameraSensor.TagIsTracked(tag) && m_backLeftCameraSensor.GetTimestamp(tag) > (units::second_t)0.0) {
      #ifdef DEBUGPOSEESTIMATION
      double startEstiamtionTime2 = (double)m_timer.GetFPGATimestamp();
      #endif
      
      m_poseEstimator.AddVisionMeasurement(m_backLeftCameraSensor.GetFieldRelativePose(tag).ToPose2d(), m_backLeftCameraSensor.GetTimestamp(tag), m_backLeftCameraSensor.GetStandardDeviations(tag));
      
      #ifdef DEBUGPOSEESTIMATION
      nte_debugTimeForAddVistionData.SetDouble((double)m_timer.GetFPGATimestamp() - startEstiamtionTime2);
      numberOfValidTags++;
      #endif
    }

    // Back Right Camera
    if (m_backRightCameraSensor.TagIsTracked(tag) && m_backRightCameraSensor.GetTimestamp(tag) > (units::second_t)0.0) {
      #ifdef DEBUGPOSEESTIMATION
      double startEstiamtionTime2 = (double)m_timer.GetFPGATimestamp();
      #endif
      
      m_poseEstimator.AddVisionMeasurement(m_backRightCameraSensor.GetFieldRelativePose(tag).ToPose2d(), m_backRightCameraSensor.GetTimestamp(tag), m_backRightCameraSensor.GetStandardDeviations(tag));
      
      #ifdef DEBUGPOSEESTIMATION
      nte_debugTimeForAddVistionData.SetDouble((double)m_timer.GetFPGATimestamp() - startEstiamtionTime2);
      numberOfValidTags++;
      #endif
    }
  }
#ifdef DEBUGPOSEESTIMATION
  nte_numberOfTagsAdded.SetInteger(numberOfValidTags);
  nte_debugTimeForPoseEstimation.SetDouble((double)m_timer.GetFPGATimestamp() - startEstiamtionTime);
#endif
} 

int DriveSubsystem::GetBestNoteId() {
  // Step 1: Get List of Notes and Robots
  m_listOfNotes.clear();
  m_listOfRobots.clear();
  for (int i = 0; i < 16; i++) {
    if (m_OakDLiteCameraSensor.ObjectIsNote(i) && m_OakDLiteCameraSensor.ObjectIsTracked(i))
      m_listOfNotes.push_back(i);
    else if (!m_OakDLiteCameraSensor.ObjectIsNote(i) && m_OakDLiteCameraSensor.ObjectIsTracked(i))
      m_listOfRobots.push_back(i);    
  }

  // Step 2: (TODO) eleminate any notes too close to robots

  // Step 3: Determine Closest Note and update class value
  if (m_listOfNotes.size() > 0) {
    double minDistanceFromRobot = 100000000.0;
    for (int i = 0; i < (int)m_listOfNotes.size(); i++) {
      if (m_OakDLiteCameraSensor.GetDistanceFromRobot(i) < minDistanceFromRobot) {
        minDistanceFromRobot = m_OakDLiteCameraSensor.GetDistanceFromRobot(i);
        m_bestNoteId = i;
      }
    }
    // return the good note
    return m_bestNoteId;
  }
  // If no Notes, return -1
  else
    return -1;
}

// Functions that pass values from sensor forward. 
// TODO: Rewrite some of these to process in a sepretate subsystem - vision
frc::Translation2d DriveSubsystem::GetRobotRelativeTranslation(int object) {
  return m_OakDLiteCameraSensor.GetRobotRelativeTranslation(object);
}

frc::Translation2d DriveSubsystem::GetFieldRelativeTranslation(int object) {
  return m_OakDLiteCameraSensor.GetFieldRelativeTranslation(object);
}

frc::Translation2d DriveSubsystem::GetRobotTranslationFieldReleative(int object) {
  return m_OakDLiteCameraSensor.GetRobotTranslationFieldReleative(object);
}

double DriveSubsystem::GetDistanceFromRobot(int object) {
  return m_OakDLiteCameraSensor.GetDistanceFromRobot(object);
}
/*
frc2::CommandPtr DriveSubsystem::DriveToAmp() {
    // Create poses based on robot position and where the goal is
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
    m_bezierPoints = {GetPose().Translation(), frc::Translation2d{GameConstants::kRobotPoseForBlueAmp.Translation().X(), (GameConstants::kRobotPoseForBlueAmp.Translation().Y() - (units::meter_t)1.0)}, 
                      frc::Translation2d{GameConstants::kRobotPoseForBlueAmp.Translation().X(),(GameConstants::kRobotPoseForBlueAmp.Translation().Y() - (units::meter_t)1.0)},  frc::Translation2d{GameConstants::kRobotPoseForBlueAmp.Translation().X(), (GameConstants::kRobotPoseForRedAmp.Translation().Y())}};
    //m_fieldPoses = {GetPose(), GameConstants::kRobotPoseForBlueAmp};
  }
  else {
    m_bezierPoints = {GetPose().Translation(), frc::Translation2d{GameConstants::kRobotPoseForRedAmp.Translation().X(), (GameConstants::kRobotPoseForRedAmp.Translation().Y() - (units::meter_t)1.0)}, 
                      frc::Translation2d{GameConstants::kRobotPoseForBlueAmp.Translation().X(), (GameConstants::kRobotPoseForRedAmp.Translation().Y() - (units::meter_t)1.0)}, frc::Translation2d{GameConstants::kRobotPoseForBlueAmp.Translation().X(), (GameConstants::kRobotPoseForRedAmp.Translation().Y())}};
    //m_fieldPoses = {GetPose(), GameConstants::kRobotPoseForRedAmp};
  }

  // Create path
  auto path = std::make_shared<pathplanner::PathPlannerPath>(
      m_bezierPoints,
      pathplanner::PathConstraints(2.5_mps, 3.0_mps_sq, 360_deg_per_s, 720_deg_per_s_sq), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
      pathplanner::GoalEndState(0.0_mps, frc::Rotation2d{(units::radian_t)-std::numbers::pi/2}) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  );

  // create bezier points out of them
  return pathplanner::AutoBuilder::followPathWithEvents(path);
}

frc2::CommandPtr DriveSubsystem::VisionIntakePath() {
  // TODO: If no note is good, craete a dummy path to follow so we are happy and don't try to ask for a note position that dosn't exsist
  // Create poses based on robot position and where the goal is
  // run command to get best note id and grab the pose at the same time
  m_fieldPoses = {GetOdometryPose(), frc::Pose2d{GetFieldRelativeTranslation(GetBestNoteId()), GetOdometryPose().Rotation()}};

  auto path = std::make_shared<pathplanner::PathPlannerPath>(
      pathplanner::PathPlannerPath::bezierFromPoses(m_fieldPoses),
      pathplanner::PathConstraints(3.5_mps, 3.5_mps_sq, 360_deg_per_s, 720_deg_per_s_sq), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
      pathplanner::GoalEndState(1.0_mps, GetPose().Rotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  );

  // create bezier points out of them
  return pathplanner::AutoBuilder::followPathWithEvents(path);
}
*/
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Utility math functions
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
double DriveSubsystem::SignedSquare(double input) {
  if (input < 0.0)
    return -std::pow(input, 2);
  else
    return std::pow(input, 2);
}

frc::Translation2d DriveSubsystem::TranslationToGoal(frc::Pose2d robotPose) {
  
  // Determine the speaker position based on allience color
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
    // Get position of center of blue speaker
    centerOfSpeaker = fieldLayout.GetTagPose(7).value().ToPose2d(); 
  } else {
    // Get position of center of red speaker
    centerOfSpeaker = fieldLayout.GetTagPose(4).value().ToPose2d(); 
  }
  
  // Find Translation of robot
  return robotPose.operator-(centerOfSpeaker).Translation();
}

double DriveSubsystem::RobotDistanceToGoal(frc::Pose2d robotPose) {
  //Find the distance between the robot and the goal
  return std::sqrt(std::pow((double)TranslationToGoal(robotPose).X(), 2) + std::pow((double)TranslationToGoal(robotPose).Y(), 2));
}

frc::Rotation2d DriveSubsystem::AngleToGoal(frc::Translation2d targetTranslation) {
  // do math
  return frc::Rotation2d{(units::radian_t)std::atan(((double)targetTranslation.Y())/((double)targetTranslation.X()))}.operator+((units::degree_t)180.0);
}