// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/current.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFields.h>
#include <rev/CANSparkMax.h>


#pragma once

// Turn this off when there is no new constants need to be burned onto motorcontrollers
#define BURNSPARKMAX 

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace RobotConstants {

const units::meter_t kIntakeSideWidth =
  0.825_m;  // Distance between centers of right and left wheels on robot
const units::meter_t kShooterSideWidth =
  0.370_m;  // Distance between centers of right and left wheels on robot
const units::meter_t kWheelBase =
  0.500_m;  // Distance between centers of front and back wheels on robot
}

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 4.0_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{1.25 * std::numbers::pi};

constexpr double kDirectionSlewRate = 4.0;   // radians per second
constexpr double kMagnitudeSlewRate = 2.0;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 4.0;  // percent per second (1 = 100%)

// CAN Sparkmax id numbers
constexpr int kFrontLeftDriveMotorPort = 22;
constexpr int kFrontRightDriveMotorPort = 17;
constexpr int kBackLeftDriveMotorPort = 21;
constexpr int kBackRightDriveMotorPort = 18;

constexpr int kFrontLeftTurningMotorPort = 23;
constexpr int kFrontRightTurningMotorPort = 16;
constexpr int kBackLeftTurningMotorPort = 20;
constexpr int kBackRightTurningMotorPort = 19;

// Anolog input ports on roborio
constexpr int kFrontLeftTurningEncoderPort = kFrontLeftTurningMotorPort;
constexpr int kFrontRightTurningEncoderPort = kFrontRightTurningMotorPort;
constexpr int kBackLeftTurningEncoderPort = kBackLeftTurningMotorPort;
constexpr int kBackRightTurningEncoderPort = kBackRightTurningMotorPort;

// Offsets in radians for the encoders. the first number to to make zero forward, after that we
// subtract an additional pi to make the full range -pi to pi instead of 0 to 2pi
constexpr double kFrontLeftDriveEncoderOffset = (1.9249 - (std::numbers::pi / 2));
constexpr double kFrontRightDriveEncoderOffset = (3.2676); 
constexpr double kBackLeftDriveEncoderOffset =  (2.0477 + std::numbers::pi); //(0.6988 + (std::numbers::pi / 2)); 
constexpr double kBackRightDriveEncoderOffset = (3.8439 + (std::numbers::pi / 2)); //(2.0472 + (std::numbers::pi)); 

constexpr auto kDriveBaseRadius = 0.46_m;

}  // namespace DriveConstants

namespace ModuleConstants {
// Through-hole Encoder on Spark MAX frequency-pwm input
// Invert the turning encoder, since the output shaft rotates in the opposite
// direction of the steering motor in the MAXSwerve Module.
constexpr bool kTurningEncoderInverted = true;

// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 13;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction =
    (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;

constexpr double kDrivingEncoderPositionFactor =
    (kWheelDiameter.value() * std::numbers::pi) /
    kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor =
    ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /
    60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor =
    (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =
    (2 * std::numbers::pi) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
    units::radian_t{kTurningEncoderPositionFactor};

constexpr double kDrivingP = 0.04;
constexpr double kDrivingI = 0;
constexpr double kDrivingD = 0;
constexpr double kDrivingFF = (1 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 1;
constexpr double kTurningI = 0;
constexpr double kTurningD = 0;
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::CANSparkMax::IdleMode kDrivingMotorIdleMode =  rev::CANSparkMax::IdleMode::kBrake;
constexpr rev::CANSparkMax::IdleMode kTurningMotorIdleMode = rev::CANSparkMax::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 50_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 20_A;

constexpr auto kModuleMaxAngularVelocity =  std::numbers::pi * 9_rad_per_s;  // radians per second ?????????
constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 20_rad_per_s / 1_s;  // radians per second^2
constexpr auto kModuleMaxLinearVelocity = 4.45_mps;
}  // namespace ModuleConstants

namespace IntakeConstants {
// Intake motor 
constexpr int kMotorID = 24;
constexpr rev::CANSparkMaxLowLevel::MotorType kMotorType = rev::CANSparkMaxLowLevel::MotorType::kBrushless;
} // namespace IntakeConstants

namespace ShooterConstants {
// Intake motor 
constexpr int kShooterID = 27;
constexpr int kFollowerID = 28;
constexpr int kPitchID = 29;
constexpr rev::CANSparkMaxLowLevel::MotorType kShooterMotorType = rev::CANSparkMaxLowLevel::MotorType::kBrushless;
constexpr rev::CANSparkMaxLowLevel::MotorType kFollowerMotorType = rev::CANSparkMaxLowLevel::MotorType::kBrushless;
constexpr rev::CANSparkMaxLowLevel::MotorType kPitchMotorType = rev::CANSparkMaxLowLevel::MotorType::kBrushless;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kShooterMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM

// Setup conversion factor for shooter encoders
constexpr double kShooterGearboxRatio = 1.0;
constexpr double kShooterPositionFactor = 1 / kShooterGearboxRatio; // revolutions
constexpr double kShooterEncoderVelocityFactor = kShooterPositionFactor;  // revolutions per minute

// Pitch encoder
constexpr double kPitchPositionFactor = (std::numbers::pi * 2); // radians
constexpr double kPitchEncoderVelocityFactor = (2 * std::numbers::pi) / 60.0; // radians per second

// PID Constants for the speed of the shooter
constexpr double kShooterP = 1.0; 
constexpr double kShooterI = 0.0; 
constexpr double kShooterD = 0.0; 
constexpr double kShooterFF = 1.0 / kShooterMotorFreeSpeedRps;
constexpr double kShooterMin = -1.0;
constexpr double kShooterMax = 1.0;

// PID Constants for the pitch of shooter
constexpr double kPitchP = 1.0;
constexpr double kPitchI = 0.0;
constexpr double kPitchD = 0.0;
constexpr double kPitchFF = 0.0;
constexpr double kPitchMin = -1.0;
constexpr double kPitchMax = 1.0;

constexpr units::ampere_t kShooterMotorCurrentLimit = 50_A;
constexpr units::ampere_t kFollowerMotorCurrentLimit = 50_A;
constexpr units::ampere_t kPitchMotorCurrentLimit = 20_A;

} // namespace IntakeConstants

namespace ElevatorConstants {
// Elevator motor 
constexpr int kElevatorID = 6;
constexpr int kFollowerID = 5;
constexpr rev::CANSparkMaxLowLevel::MotorType kElevatorMotorType = rev::CANSparkMaxLowLevel::MotorType::kBrushless;
constexpr rev::CANSparkMaxLowLevel::MotorType kFollowerMotorType = rev::CANSparkMaxLowLevel::MotorType::kBrushless;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kElevatorMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM

// Setup conversion factor for Elevator encoders
constexpr double kElevatorGearboxRatio = 4.0;
constexpr double kSpoolDiameter = 7.62; // meters
constexpr double kElevatorPositionFactor = (std::numbers::pi * kSpoolDiameter) / kElevatorGearboxRatio; // meters
constexpr double kElevatorEncoderVelocityFactor = kElevatorPositionFactor / 60.0;  // meters per second

// PID Constants for the pitch of the Elevator
constexpr double kElevatorP = 1.0; 
constexpr double kElevatorI = 0.0; 
constexpr double kElevatorD = 0.0; 
constexpr double kElevatorFF = 0.0;
constexpr double kElevatorMin = -1.0;
constexpr double kElevatorMax = 1.0;

constexpr units::ampere_t kElevatorMotorCurrentLimit = 50_A;
constexpr units::ampere_t kFollowerMotorCurrentLimit = 50_A;

} // namespace IntakeConstants

namespace AutoConstants {

// Only Constants here are the PID constants. Look in path planner for max veleocty/acceleration constants
// PID Constants for the tranlation (X and Y movement) of the robot during auto
constexpr double kPTanslationController = 4.0; // 6.0
constexpr double kITanslationController = 0.0; // 1.7
constexpr double kDTanslationController = 0.0; // 0.0

// PID Constants for the rotation, or Yaw of the robot
constexpr double kPRotationController = 5.0; // 5.0
constexpr double kIRotationController = 0.0; // 0.0
constexpr double kDRotationController = 0.0; // 0.0

}  // namespace AutoConstants

namespace ControllerConstants {

// Controller Constants for the flight elite drive controller

// Axis indexes
constexpr int kDriveLeftYIndex = 1; // An input UP creates a NEGATIVE output
constexpr int kDriveLeftXIndex = 0; // An input RIGHT creates a NEGATIVE output
constexpr int kDriveRightYIndex = 5; // An input UP creates a NEGATIVE output
constexpr int kDriveRightXIndex = 4; // An input RIGHT creates a NEGATIVE output

constexpr int kOperatorLeftYIndex = 1; // An input UP creates a NEGATIVE output

// Drive Controller
constexpr int kFieldRelativeButtonIndex = 7; // CL
constexpr int kRobotRelativeButtonIndex = 8; // CR
constexpr int kSlowStateButtonIndex = 4; // Y
constexpr int kFastStateButtonIndex = 1; // A
constexpr int kResetGyroButtonIndex = 2; // B

// Operator controller 
constexpr int kIntakeButtonIndex = 8; // RT
constexpr int kOuttakeButtonIndex = 7; // LT
constexpr int kShooterButtonIndex = 5; // LB
} // namespace ControllerConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kOperatorControllerPort = 1;
}  // namespace OIConstants

namespace CameraConstats {

// Pose3d/transformation2d of the camera relative to the robot
// X if forward, Y is Left, Z is up 
namespace FrontCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)0.2286, (units::meter_t)0.0, (units::meter_t)0.1651};
    const frc::Rotation3d kRotation3d{(units::radian_t)0.0, (units::radian_t)0.0, (units::radian_t)0.0};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
} // namespace FrontCamera

namespace BackCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)-0.2667, -(units::meter_t)0.0, (units::meter_t)0.0635};
    const frc::Rotation3d kRotation3d{(units::radian_t)0.0, (units::radian_t)0.0, (units::radian_t)std::numbers::pi};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
} // namespace BackCamera


} // namespace CameraConstants

namespace DataCurve {
  /**
   * @param distance The distance from the goal to the robot
   * 
   * @return The angle in radians based off of the curve calculations
  */
  double GetAngleFromDistance(double distance);

  /**
   * @param distance The distance from the goal to the robot
   * 
   * @return The speed in RPM based off of the curve calculations
  */
  double GetSpeedFromDistance(double distance);

  /**
   * @param distance The distance from the goal to the robot
   * 
   * @return The time in seconds based off of the curve calculations
  */
  double GetTimeFromDistnace(double Angle);
}