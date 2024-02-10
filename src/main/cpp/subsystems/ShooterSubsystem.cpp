// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"
#include "Constants.h"


ShooterSubsystem::ShooterSubsystem() : m_shooterMotor{ShooterConstants::kShooterID, ShooterConstants::kShooterMotorType},
                    m_shooterFollower{ShooterConstants::kFollowerID, ShooterConstants::kFollowerMotorType},
                    m_pitchMotor{ShooterConstants::kPichID, ShooterConstants::kPichMotorType}{
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr ShooterSubsystem::ExampleMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool ShooterSubsystem::ExampleCondition() {
  // Query some boolean state, such as a digital sensor.
  return false;
}

void ShooterSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ShooterSubsystem::SetMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_shooterMotor.Set(power);
}