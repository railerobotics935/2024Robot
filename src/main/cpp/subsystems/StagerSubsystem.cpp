// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/StagerSubsystem.h"
#include "Constants.h"


StagerSubsystem::StagerSubsystem() : m_stagerMotor{StagerConstants::kMotorID, StagerConstants::kMotorType},
                                     m_stagerFollower{StagerConstants::kFollowerID, StagerConstants::kMotorType}{
  // Implementation of subsystem constructor goes here.

  #ifdef BURNSTAGERSPARKMAX
  // Restore to deafaults
  m_stagerMotor.RestoreFactoryDefaults();
  m_stagerFollower.RestoreFactoryDefaults();

  // Set follwers and inversted state
  m_stagerFollower.SetInverted(true);

  printf("Flash Burned on shooter subsystem\r\n");
  #else
  printf("Flash was not burned on shooter subsystem\r\n");
  #endif

  m_stagerFollower.Follow(m_stagerMotor);
}

void StagerSubsystem::SetMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_stagerMotor.Set(power);
}