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
#include <units/voltage.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFields.h>


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

const units::meter_t kTrackWidth =
  0.5_m;  // Distance between centers of right and left wheels on robot
const units::meter_t kWheelBase =
  0.7_m;  // Distance between centers of front and back wheels on robot

}

namespace DriveConstants {

// CAN Sparkmax id numbers
constexpr int kFrontLeftDriveMotorPort = 11;
constexpr int kFrontRightDriveMotorPort = 9;
constexpr int kBackLeftDriveMotorPort = 19;
constexpr int kBackRightDriveMotorPort = 21;

constexpr int kFrontLeftTurningMotorPort = 12;
constexpr int kFrontRightTurningMotorPort = 10;
constexpr int kBackLeftTurningMotorPort = 20;
constexpr int kBackRightTurningMotorPort = 2;

// Anolog input ports on roborio
constexpr int kFrontLeftTurningEncoderPort = 0;
constexpr int kFrontRightTurningEncoderPort = 1;
constexpr int kBackLeftTurningEncoderPort = 2;
constexpr int kBackRightTurningEncoderPort = 3;

// Offsets in radians for the encoders. the first number to to make zero forward, after that we
// subtract an additional pi to make the full range -pi to pi instead of 0 to 2pi
constexpr double kFrontLeftDriveEncoderOffset = 1.113 + std::numbers::pi;
constexpr double kFrontRightDriveEncoderOffset = 2.787 + std::numbers::pi;
constexpr double kBackLeftDriveEncoderOffset =  -0.155 + std::numbers::pi;
constexpr double kBackRightDriveEncoderOffset = -1.850 + std::numbers::pi;

constexpr auto kRobotMaxLinearVelocity = 3.0_mps; // 4.6
constexpr auto kRobotMaxAngularVelocity = std::numbers::pi * 8_rad_per_s;

constexpr auto kDriveBaseRadius = 0.46_m;

}  // namespace DriveConstants

namespace ModuleConstants {
// This is something to try and get rid of
// If feels like there should be something like this built into the systems
constexpr double ANALOG_TO_RAD_FACTOR = 1.2566;     // 0 to 5.0 volt = 2PI rad
constexpr double SPARK_MAX_ANALOG_TO_RAD_FACTOR = 1.9040;     // 0 to 3.3 volt = 2PI rad  should come back and revisit this to maybe fix anolog things.

constexpr double kWheelRadiusMeters = 0.0508;
constexpr int kEncoderResolution = 42;
constexpr double kGearRatio = 8.14;

constexpr double kDriveVelocityEncoderConversionFactor = (2.0 * std::numbers::pi * kWheelRadiusMeters 
                                                / (kGearRatio * kEncoderResolution));

constexpr double kDrivePositionEncoderConversionFactor = (2.0 * std::numbers::pi * kWheelRadiusMeters 
                                                / (kGearRatio));

constexpr auto kModuleMaxAngularVelocity =  std::numbers::pi * 9_rad_per_s;  // radians per second ?????????
constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 20_rad_per_s / 1_s;  // radians per second^2
constexpr auto kModuleMaxLinearVelocity = 3.81_mps;

constexpr double kPModuleTurningController = 1;

constexpr double kPModuleDriveController = 1;
constexpr double kIModuleDriveController = 0.1;
constexpr double kDModuleDriveController = 0.1;
}  // namespace ModuleConstants

namespace AutoConstants {

// Only Constants here are the PID constants. Look in path planner for max veleocty/acceleration constants
// PID Constants for the tranlation (X and Y movement) of the robot during auto
constexpr double kPTanslationController = 4; // 6.0
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
const frc::Translation3d kFrontCameraTranlation3d{(units::meter_t)0.0, (units::meter_t)RobotConstants::kTrackWidth / 2, (units::meter_t)0.4064};
const frc::Rotation3d kFrontCameraRotation3d{(units::radian_t)0.0, (units::radian_t)0.0, (units::radian_t)0.0};
const frc::Pose3d kFrontCameraPose3d{kFrontCameraTranlation3d, kFrontCameraRotation3d};
const frc::Transform2d kFrontCameraTransform2d{kFrontCameraPose3d.ToPose2d().Translation(), kFrontCameraPose3d.ToPose2d().Rotation()};

} // namespace CameraConstants