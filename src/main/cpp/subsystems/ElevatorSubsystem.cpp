// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"
#include "Constants.h"


ElevatorSubsystem::ElevatorSubsystem() : m_elevatorMotor{ElevatorConstants::kElevatorID, ElevatorConstants::kElevatorMotorType} {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr ElevatorSubsystem::ExampleMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool ElevatorSubsystem::ExampleCondition() {
  // Query some boolean state, such as a digital sensor.
  return false;
}

void ElevatorSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

