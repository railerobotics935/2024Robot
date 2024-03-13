// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"
#include "Constants.h"
#include <networktables/NetworkTableInstance.h>

using namespace ClimberConstants;

ClimberSubsystem::ClimberSubsystem(int motorId, int limitSwitchPort, bool reversed) : m_climberMotor{motorId, kMotorType}, m_limitSwitch{limitSwitchPort} {

  // Burn flash only if desired - true set in constants
  #ifdef BURNCLIMBERSPARKMAX
  // Restore deafults
  m_climberMotor.RestoreFactoryDefaults();

  // Set converstion factors for encoders
  m_climberEncoder.SetPositionConversionFactor(kPositionFactor);
  m_climberEncoder.SetVelocityConversionFactor(kVelocityFactor);

  // Set Idle mode (what to do when not commanded at a speed)
  m_climberMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  m_climberMotor.SetSmartCurrentLimit(kMotorCurrentLimit.value());
  if (reversed)
    m_climberMotor.SetInverted(true);
  else
    m_climberMotor.SetInverted(false);

  m_climberMotor.BurnFlash();

  printf("Flash Burned on climber subsystem\r\n");
  #else
  printf("Flash was not burned on climber subsystem\r\n");
  #endif
}

bool ClimberSubsystem::ExampleCondition() {
  // Query some boolean state, such as a digital sensor.
  return false;
}
 /**
  * 
  * 
  * 
  * 
  * LIMITSWITCH IS REVERSED
  * 
  * 
  * 
  * 
  * 
 */
void ClimberSubsystem::Periodic() {
  if (m_limitSwitch.Get())
    m_climberEncoder.SetPosition(0.0);
}

void ClimberSubsystem::SetClimberPower(double power) {
  if (!
  
  m_limitSwitch.Get()){
    m_climberMotor.Set(power);
  }
  else
    printf("Climber At Switch\r\n");
}

