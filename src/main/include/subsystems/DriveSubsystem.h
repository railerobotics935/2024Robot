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
#include <frc/controller/PIDController.h>
#include <frc2/command/PIDSubsystem.h>
#include <frc/Timer.h>

#include "Constants.h"
#include "SwerveModule.h"
#include "sensors/ApriltagSensor.h"
#include "sensors/OakDLiteSensor.h"

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

  /**
   * Updates all of the network table entries
  */
  void UpdateNTE();

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds haven the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param rateLimit     Whether to limit the rate of the robot (Recomended)
   */
  void Drive(units::meters_per_second_t xSpeed,
              units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
              bool rateLimit);

  /**
   * Drives the robot at given x and y, and faces the angle given.  
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotation           Angular rate of the robot.
   * @param rateLimit     Whether to limit the rate of the robot (Recomended)
  */
  void DriveFacingGoal(units::meters_per_second_t xSpeed,
                        units::meters_per_second_t ySpeed, frc::Rotation2d rotation,
                        bool rateLimit);
  
  /**
   * Gets if the PID Controller is at the setpoint
   * 
   * @return True if the robot is at the correct angle setpoint 
  */
  bool AtAngleSetpoint();

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
   * @return The speed of the robot
  */
  double GetLinearRobotSpeed();

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Sets the robot to Robot relative mode
  */
  void SetRobotRelative();

  /**
   * Sets the robot to Field relative mode
  */
  void SetFieldRelative();

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
   * Returns the rotation of the robot reletive to the driver to use for field relative
   *
   * @return The rotation.
   */
  frc::Rotation2d GetRotation();

  /**
   * Returns the chassis speeds of the robot IN ROBOT RELATIVE
   * 
   * @return Robot chassis speeds
  */
  frc::ChassisSpeeds GetRobotRelativeSpeeds();

  /**
   * Returns the chassis speeds of the robot IN FIELD RELATIVE
   * 
   * @return Robot chassis speeds
  */
  frc::ChassisSpeeds GetFieldRelativeSpeeds();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  /**
   *  Uses the 3d transfomation information from apriltag to further 
   *  update the position of the robot
  */
  void EstimatePoseWithApriltag();

//------------------------------------------------------------------------------------------------------------------------------------
// Oak-D Lite Camera Sensor helper methods
//------------------------------------------------------------------------------------------------------------------------------------
  /**
   * Used internaly to determine whitch note the camera may see to travel too
   * 
   * @returns the id of the best note
  */
  int GetBestNoteId();

  /**
   * @param object The ID number for the object wanted to identify
   * @return The Robot Relative Translation 2d
  */
  frc::Translation2d GetRobotRelativeTranslation(int object);

  /**
   * @param object The ID number for the object wanted to identify
   * @return The Field Relative Translation based on pose estimation
  */
  frc::Translation2d GetFieldRelativeTranslation(int object);

  /**
   * @param object The ID number for the object wanted to identify
   * @return The Note Translation to the robot relative 
  */
  frc::Translation2d GetRobotTranslationFieldReleative(int object);

  /**
   * @param object The ID nubmer for the object wanted to identify
   * @return The distance in Meters the object is from the center fo the robot
  */
  double GetDistanceFromRobot(int object);

//------------------------------------------------------------------------------------------------------------------------------------
// Math Utils Helper Methods
//------------------------------------------------------------------------------------------------------------------------------------

  /**
   * Takes an input value and squares it, but retains the sign. IE negative
   * numbers will remain negative.
   * 
   * @param input is the number to perform the transform on
   * @return the transformed input value
  */
  double SignedSquare(double input);

  /**
   * Finds transformation of robot in relation to the goal. 
   * 
   * @param robotPose is the position of the robot on the field
   * @return the transformation of the robot
  */
  frc::Translation2d TranslationToGoal(frc::Pose2d robotPose);

  /**
   * Turns translation2d given by TranslationToGoal to distance between the robot and 
   * goal in meters.
   * 
   * @param robotTransformation is the transformation given by TranslationToGoal
   * @return the distance to the goal in meters
  */
  double RobotDistanceToGoal(frc::Pose2d robotPose);

  /**
   * Finds the angle of the robot in relation to the goal. 
   * 
   * @param targetTranslation is the position of the robot on the field
   * @return the rotation of the robot
  */
  frc::Rotation2d AngleToGoal(frc::Translation2d targetTranslation);

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

  nt::NetworkTableEntry nte_kp;
  nt::NetworkTableEntry nte_ki;
  nt::NetworkTableEntry nte_kd;

  nt::NetworkTableEntry nte_robot_distance_to_goal;

  nt::NetworkTableEntry nte_debugTimeForPoseEstimation;
  nt::NetworkTableEntry nte_debugTimeForAddVistionData;
  nt::NetworkTableEntry nte_numberOfTagsAdded;

  frc::Field2d m_field;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::Timer m_timer;

  SwerveModule m_frontLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backLeft;
  SwerveModule m_backRight;

  // The gyro sensor
  frc::ADIS16470_IMU m_gyro{frc::ADIS16470_IMU::IMUAxis::kZ, frc::ADIS16470_IMU::IMUAxis::kY, frc::ADIS16470_IMU::IMUAxis::kX};
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

  // PID controller for robot angle
  frc::PIDController m_robotAngleController{DriveConstants::kRotationP, DriveConstants::kRotationI, DriveConstants::kRotationD};

  // Variables to internialy keep track of drive state
  bool m_fieldRelative = true;

  // Create path to deploy directory
  fs::path deployDirectory{frc::filesystem::GetDeployDirectory() + "/2024-crescendo.json"};

  // Initialize variables
  frc::AprilTagFieldLayout fieldLayout{deployDirectory.string()}; 
  frc::Pose2d centerOfSpeaker{};

  // Apriltag sensor 
  ApriltagSensor m_frontCameraSensor{"FrontCam", CameraConstants::FrontCamera::kPose3d};
  ApriltagSensor m_backLeftCameraSensor{"BackLeftCam", CameraConstants::BackLeftCamera::kPose3d};
  ApriltagSensor m_backRightCameraSensor{"BackRightCam", CameraConstants::BackRightCamera::kPose3d};
  OakDLiteSensor m_OakDLiteCameraSensor{"OakDLiteCam", CameraConstants::OakDLiteCamera::kPose3d, &m_poseEstimator};

  // Processing variables for the 
  int m_bestNoteId;
  std::vector<int> m_listOfNotes;
  std::vector<int> m_listOfRobots;
};
