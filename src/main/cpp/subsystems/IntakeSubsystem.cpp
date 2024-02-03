// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"


IntakeSubsystem::IntakeSubsystem() : m_intakeMotor{IntakeConstants::kMotorID, IntakeConstants::kMotorType} {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr IntakeSubsystem::ExampleMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool IntakeSubsystem::ExampleCondition() {
  // Query some boolean state, such as a digital sensor.
  return false;
}

void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void IntakeSubsystem::TurnOn() {
  // Turn on the motor. 
  m_intakeMotor.Set(1.0);
}

void IntakeSubsystem::TurnOff() {
  // Turn off the motor.
  m_intakeMotor.Set(0.0);
}

void IntakeSubsystem::SetMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_intakeMotor.Set(power);
}