// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/interfaces/Gyro.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include "Constants.h"
#include "SwerveModule.h"
#include "ApriltagSensor.h"

class DriveSubsystem : public frc2::SubsystemBase {
public:
  DriveSubsystem();
  /**
   * Will return true when the robot is on the red alliance. Specificly
   * for the pathplanner path orientaion
   */
  bool InRedAlliance();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds haven the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   */
  void Drive(units::meters_per_second_t xSpeed,
              units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
              bool rateLimit);

  /**
   * Drives the robot given a set of chasis speeds IN ROBOT RELATIVE
   * 
   * @param speeds        the Chasis speed of the robot 
   *                      (Xspeed, Yspeed, ROTspeed)
  */
  void DriveWithChassisSpeeds(frc::ChassisSpeeds speeds);

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, From Something to something
   */
  units::degree_t GetHeading() const;

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  void SetRobotRelative();

  void SetFieldRelative();

  void SetSlowMode();

  void SetFastMode();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Returns the chassis speeds of the robot IN ROBOT RELATIVE
   * 
   * @return Robot chassis speeds
  */
  frc::ChassisSpeeds GetRobotRelativeSpeeds();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  /**
   * Sets the wheels on the swerve modules in an X shape 
   * 
  */
  void Park();

  /**
   *  Uses the 3d transfomation information from apriltag to further 
   *  update the position of the robot
  */
  void EstimatePoseWithApriltag();

  frc2::CommandPtr ConfigureModuleControllers();

  frc::SwerveDriveKinematics<4> m_driveKinematics{
    frc::Translation2d{RobotConstants::kWheelBase / 2, RobotConstants::kShooterSideWidth / 2},
    frc::Translation2d{RobotConstants::kWheelBase / 2, -RobotConstants::kShooterSideWidth / 2},
    frc::Translation2d{-RobotConstants::kWheelBase / 2, RobotConstants::kIntakeSideWidth / 2},
    frc::Translation2d{-RobotConstants::kWheelBase / 2, -RobotConstants::kIntakeSideWidth / 2}};

private:
  // Declaring all of the network table entries
  nt::NetworkTableEntry nte_fl_set_angle;
  nt::NetworkTableEntry nte_fr_set_angle;
  nt::NetworkTableEntry nte_bl_set_angle;
  nt::NetworkTableEntry nte_br_set_angle;
  nt::NetworkTableEntry nte_fl_set_speed;
  nt::NetworkTableEntry nte_fr_set_speed;
  nt::NetworkTableEntry nte_bl_set_speed;
  nt::NetworkTableEntry nte_br_set_speed;
  
  nt::NetworkTableEntry nte_fl_real_angle;
  nt::NetworkTableEntry nte_fr_real_angle;
  nt::NetworkTableEntry nte_bl_real_angle;
  nt::NetworkTableEntry nte_br_real_angle;
  nt::NetworkTableEntry nte_fl_real_speed;
  nt::NetworkTableEntry nte_fr_real_speed;
  nt::NetworkTableEntry nte_bl_real_speed;
  nt::NetworkTableEntry nte_br_real_speed;

  nt::NetworkTableEntry nte_fl_raw_encoder_voltage;
  nt::NetworkTableEntry nte_fr_raw_encoder_voltage;
  nt::NetworkTableEntry nte_bl_raw_encoder_voltage;
  nt::NetworkTableEntry nte_br_raw_encoder_voltage;

  nt::NetworkTableEntry nte_gyro_angle;
  nt::NetworkTableEntry nte_robot_x;
  nt::NetworkTableEntry nte_robot_y;

  nt::NetworkTableEntry nte_robot_distance_to_goal;

  frc::Field2d m_field;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  SwerveModule m_frontLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backLeft;
  SwerveModule m_backRight;

  // The gyro sensor
  frc::ADIS16470_IMU m_gyro{frc::ADIS16470_IMU::IMUAxis::kZ, frc::ADIS16470_IMU::IMUAxis::kY, frc::ADIS16470_IMU::IMUAxis::kX};

  // Apriltag sensor 
  ApriltagSensor m_frontCameraSensor{"FrontCam", CameraConstats::FrontCamera::kPose3d};
  ApriltagSensor m_backCameraSensor{"BackCam", CameraConstats::BackCamera::kPose3d};

  // Odometry class for tracking robot pose
  // 4 defines the number of modules
  frc::SwerveDriveOdometry<4> m_odometry;

  // Pose Estimator
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

  // Slew rate filter variables for controlling lateral acceleration
  double m_currentRotation = 0.0;
  double m_currentTranslationDir = 0.0;
  double m_currentTranslationMag = 0.0;

  frc::SlewRateLimiter<units::scalar> m_magLimiter{
      DriveConstants::kMagnitudeSlewRate / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{
      DriveConstants::kRotationalSlewRate / 1_s};
  double m_prevTime = wpi::Now() * 1e-6;

  // Variables to internialy keep track of drive state
  bool m_fieldRelative = true;
  bool m_slowMode = false;
};
