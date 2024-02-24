// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/StagerSubsystem.h"
#include "Constants.h"


StagerSubsystem::StagerSubsystem() : m_stagerMotor{IntakeConstants::kMotorID, IntakeConstants::kMotorType} {
  // Implementation of subsystem constructor goes here.
}

void StagerSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void StagerSubsystem::SetMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_stagerMotor.Set(power);
}