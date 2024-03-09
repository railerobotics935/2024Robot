// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"


IntakeSubsystem::IntakeSubsystem() : m_intakeMotor{IntakeConstants::kMotorID, IntakeConstants::kMotorType} {
  // Implementation of subsystem constructor goes here.
  #ifdef BURNSTAGERSPARKMAX
  m_intakeMotor.RestoreFactoryDefaults();

  // Enable Voltage Compensation
  m_intakeMotor.EnableVoltageCompensation(RobotConstants::kVoltageCompentationValue);

  printf("Burned Intake Motor Controller\r\n");
  #else

  printf("Did Not Burn Intake Motor Controller\r\n");
  #endif
}

void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void IntakeSubsystem::SetMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_intakeMotor.Set(power);
}
