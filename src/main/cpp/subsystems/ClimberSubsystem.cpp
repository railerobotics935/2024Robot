// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"
#include "Constants.h"
#include <networktables/NetworkTableInstance.h>

using namespace ClimberConstants;

ClimberSubsystem::ClimberSubsystem() {

  // Burn flash only if desired - true set in constants
  #ifdef BURNCLIMBERSPARKMAX
  // Restore deafults
  m_leftClimberMotor.RestoreFactoryDefaults();
  m_rightClimberMotor.RestoreFactoryDefaults();
  
  // Set converstion factors for encoders
  m_leftClimberEncoder.SetPositionConversionFactor(kPositionFactor);
  m_leftClimberEncoder.SetVelocityConversionFactor(kVelocityFactor);

  m_rightClimberEncoder.SetPositionConversionFactor(kPositionFactor);
  m_rightClimberEncoder.SetVelocityConversionFactor(kVelocityFactor);
  
  // Set Idle mode (what to do when not commanded at a speed)
  m_leftClimberMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  m_rightClimberMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  m_leftClimberMotor.SetSmartCurrentLimit(kMotorCurrentLimit.value());
  m_rightClimberMotor.SetSmartCurrentLimit(kMotorCurrentLimit.value());
  
  // Invert the left becasue its mirrored
  m_leftClimberMotor.SetInverted(true);

  m_leftClimberMotor.BurnFlash();
  m_rightClimberMotor.BurnFlash();

  printf("Flash Burned on climber subsystem\r\n");
  #else
  printf("Flash was not burned on climber subsystem\r\n");
  #endif

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("Climber");

  m_leftCllimberLimitSwtich = nt_table->GetEntry("Left Climber/Limit Switch");
  m_leftClimberDistance = nt_table->GetEntry("Left Climber/Distance Extended");
  m_rightCllimberLimitSwtich = nt_table->GetEntry("Right Climber/Limit Switch");
  m_rightClimberDistance = nt_table->GetEntry("Right Climber/Distance Extended");
}

bool ClimberSubsystem::LeftClimberAtBase() {
  return m_leftLimitSwitch.Get();
}

bool ClimberSubsystem::RightClimberAtBase() {
  return m_rightLimitSwitch.Get();
}

void ClimberSubsystem::Periodic() {
  m_leftCllimberLimitSwtich.SetBoolean(LeftClimberAtBase());
  m_leftClimberDistance.SetDouble(m_leftClimberEncoder.GetPosition());
  m_rightCllimberLimitSwtich.SetBoolean(RightClimberAtBase());
  m_rightClimberDistance.SetDouble(m_rightClimberEncoder.GetPosition());

  if (LeftClimberAtBase())
    m_leftClimberEncoder.SetPosition(0.0);
  if (RightClimberAtBase())
    m_rightClimberEncoder.SetPosition(0.0);
}

void ClimberSubsystem::SetClimberPower(double power) {
  if (power < 0.0 && m_leftClimberEncoder.GetPosition() < -6.2) {
    m_leftClimberMotor.Set(0.0);
    m_rightClimberMotor.Set(0.0);
  }
  else {
  if (!LeftClimberAtBase() || power < 0.0)
    m_leftClimberMotor.Set(power);
  else
    m_leftClimberMotor.Set(0.0);
  
  if (!RightClimberAtBase() || power < 0.0)
    m_rightClimberMotor.Set(power);
  else
    m_rightClimberMotor.Set(0.0);
  }
  
}

