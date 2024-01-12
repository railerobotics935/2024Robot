// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogInput.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include "rev/CANSparkMax.h"
#include "rev/SparkMaxAnalogSensor.h"

#include "Constants.h"

class SwerveModule {
 public:
  SwerveModule(int driveMotorPort, int turningMotorPort,
                const int turningEncoderPort, const double turningEncoderOffset);

  frc::SwerveModuleState GetState();

  frc::SwerveModulePosition GetPosition();

  void SetDesiredState(const frc::SwerveModuleState& state);

  // void ResetEncoders(); currently using absolute encoders, so you can't reset them digitaly

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.
  
  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;
  
  rev::SparkRelativeEncoder m_driveEncoder;
  frc::AnalogInput m_turningEncoder;
  
  double m_kTurningEncoderOffset;
  
  frc::PIDController m_drivePIDController{ModuleConstants::kPModuleDriveController, 0, 0};

  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      12.5, 190.0, 0.15,
      {ModuleConstants::kModuleMaxAngularVelocity, ModuleConstants::kModuleMaxAngularAcceleration}};
  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{0.5_V, 1_V / 1_mps};

};
