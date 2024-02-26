// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogInput.h>                        // for closed-loop control on RoboRIO
#include <frc/controller/PIDController.h>           // for closed-loop control on RoboRIO
#include <frc/controller/ProfiledPIDController.h>   // for closed-loop control on RoboRIO
#include <frc/controller/SimpleMotorFeedforward.h>  // for closed-loop control on robo
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <rev/SparkAbsoluteEncoder.h>               // for through-hole encoder on Spark MAX
#include <rev/SparkPIDController.h>                 // for closed-loop control on Spark MAX 
#include <rev/SparkRelativeEncoder.h>               // for closed-loop control on Spark MAX

#include "rev/CANSparkMax.h"
#include "rev/SparkAnalogSensor.h"

#include "Constants.h"

class SwerveModule {
 public:
  SwerveModule(const int drivingCANId, const int turningCANId, const double turingEncoderOffset);

  frc::SwerveModuleState GetState();

  frc::SwerveModulePosition GetPosition();

  void SetDesiredState(const frc::SwerveModuleState& state);

  /**
   * Burn configuration onto sparkmax motorcontrollers
  */
  void ConfigureSparkMax();

  // void ResetEncoders(); currently using absolute encoders, so you can't reset them digitaly

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.
  
  rev::CANSparkMax m_drivingSparkMax;
  rev::CANSparkMax m_turningSparkMax;

  rev::SparkRelativeEncoder m_drivingEncoder = m_drivingSparkMax.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkAbsoluteEncoder m_turningAbsoluteEncoder = m_turningSparkMax.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

  rev::SparkPIDController m_drivingPIDController = m_drivingSparkMax.GetPIDController();
  rev::SparkPIDController m_turningPIDController = m_turningSparkMax.GetPIDController();

  double m_turingEncoderOffset = 0.0;
  frc::SwerveModuleState m_desiredState{units::meters_per_second_t{0.0},
                                        frc::Rotation2d()};
};
