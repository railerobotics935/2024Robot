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

// CAN Sparkmax id numbers
constexpr int kFrontLeftDriveMotorPort = 1;
constexpr int kFrontRightDriveMotorPort = 3;
constexpr int kBackLeftDriveMotorPort = 5;
constexpr int kBackRightDriveMotorPort = 7;

constexpr int kFrontLeftTurningMotorPort = 2;
constexpr int kFrontRightTurningMotorPort = 4;
constexpr int kBackLeftTurningMotorPort = 6;
constexpr int kBackRightTurningMotorPort = 8;

// Anolog input ports on roborio
constexpr int kFrontLeftTurningEncoderPort = kFrontLeftTurningMotorPort;
constexpr int kFrontRightTurningEncoderPort = kFrontRightTurningMotorPort;
constexpr int kBackLeftTurningEncoderPort = kBackLeftTurningMotorPort;
constexpr int kBackRightTurningEncoderPort = kBackRightTurningMotorPort;

// Offsets in radians for the encoders. the first number to to make zero forward, after that we
// subtract an additional pi to make the full range -pi to pi instead of 0 to 2pi
constexpr double kFrontLeftDriveEncoderOffset = 0.0;
constexpr double kFrontRightDriveEncoderOffset = 0.0;
constexpr double kBackLeftDriveEncoderOffset =  0.0;
constexpr double kBackRightDriveEncoderOffset = 0.0;

constexpr auto kRobotMaxLinearVelocity = 3.0_mps; // 4.6
constexpr auto kRobotMaxAngularVelocity = std::numbers::pi * 5_rad_per_s;

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
constexpr int kDrivingMotorPinionTeeth = 14;

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

// This is something to try and get rid of
// If feels like there should be something like this built into the systems
constexpr double ANALOG_TO_RAD_FACTOR = 1.2566;     // 0 to 5.0 volt = 2PI rad
constexpr double SPARK_MAX_ANALOG_TO_RAD_FACTOR = 1.9040;     // 0 to 3.3 volt = 2PI rad  should come back and revisit this to maybe fix anolog things.

constexpr double kWheelRadiusMeters = 0.0508;
constexpr int kEncoderResolution = 42;
constexpr double kGearRatio = 8.14;

//constexpr double kDriveVelocityEncoderConversionFactor = (2.0 * std::numbers::pi * kWheelRadiusMeters 
//                                                / (kGearRatio * kEncoderResolution));

//constexpr double kDrivePositionEncoderConversionFactor = (2.0 * std::numbers::pi * kWheelRadiusMeters 
//                                                / (kGearRatio));

constexpr auto kModuleMaxAngularVelocity =  std::numbers::pi * 9_rad_per_s;  // radians per second ?????????
constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 20_rad_per_s / 1_s;  // radians per second^2
constexpr auto kModuleMaxLinearVelocity = 3.81_mps;

constexpr double kPModuleDriveController = 1;
constexpr double kIModuleDriveController = 0.1;
constexpr double kDModuleDriveController = 0.1;
}  // namespace ModuleConstants

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
constexpr int kDriveLeftYIndex = 2; // An imput UP creates a NEGATIVE output
constexpr int kDriveLeftXIndex = 4; // An imput RIGHT creates a NEGATIVE output
constexpr int kDriveRightYIndex = 1; // An imput UP creates a NEGATIVE output
constexpr int kDriveRightXIndex = 0; // An imput RIGHT creates a NEGATIVE output

// Button/Switch indexes
constexpr int kFieldRelativeSwitchIndex = 1;
constexpr int kParkSwitchIndex = 2;
constexpr int kSlowStateSwitchIndex = 5;
constexpr int kResetGyroButtonIndex = 3;

} // namespace ControllerConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants

namespace CameraConstats {

// Pose3d/transformation2d of the camera relative to the robot
// X if forward, Y is Left Z is up 
const frc::Translation3d kFrontCameraTranlation3d{(units::meter_t)0.0, (units::meter_t)RobotConstants::kShooterSideWidth / 2, (units::meter_t)0.4064};
const frc::Rotation3d kFrontCameraRotation3d{(units::radian_t)0.0, (units::radian_t)0.0, (units::radian_t)0.0};
const frc::Pose3d kFrontCameraPose3d{kFrontCameraTranlation3d, kFrontCameraRotation3d};
const frc::Transform2d kFrontCameraTransform2d{kFrontCameraPose3d.ToPose2d().Translation(), kFrontCameraPose3d.ToPose2d().Rotation()};

} // namespace CameraConstants