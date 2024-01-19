// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/Field2d.h>

#include "Constants.h"
#include "SwerveModule.h"

class DriveSubsystem : public frc2::SubsystemBase {
public:
  DriveSubsystem();
  /**
   * Will return true when the robot is on the red allience. Specificly
   * for the pathplanner path orientaion
   */
  bool InRedAlience();

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
              bool fieldRelative);

  /**
   * Drives the robot given a set of chasis speeds IN ROBOT RELATIVE
   * 
   * @param speeds        the Chasis speed of the robot 
   *                      (Xspeed, Yspeed, ROTspeed)
  */
  void DriveWithChassisSpeeds(frc::ChassisSpeeds speeds);

  /**
   * Resets the drive encoders to currently read a position of 0.
   * 
   * IMPORTAND: Currently using ablosute encoders, so you can't reset them.
   */
  void ResetEncoders();

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
  void EstimatePoseWithApriltags();

  units::meter_t kTrackWidth =
    0.5_m;  // Distance between centers of right and left wheels on robot
  units::meter_t kWheelBase =
    0.7_m;  // Distance between centers of front and back wheels on robot

  frc::SwerveDriveKinematics<4> m_driveKinematics{
    frc::Translation2d{kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{kWheelBase / 2, -kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, -kTrackWidth / 2}};

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

  frc::Field2d m_field;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  SwerveModule m_frontLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backLeft;
  SwerveModule m_backRight;

  // The gyro sensor
  frc::ADIS16470_IMU m_gyro{frc::ADIS16470_IMU::IMUAxis::kZ, frc::ADIS16470_IMU::IMUAxis::kY, frc::ADIS16470_IMU::IMUAxis::kX};

  // Odometry class for tracking robot pose
  // 4 defines the number of modules
  frc::SwerveDriveOdometry<4> m_odometry;
};
