// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/StagerSubsystem.h"
#include "Constants.h"
#include "string"


StagerSubsystem::StagerSubsystem() : m_stagerMotor{StagerConstants::kMotorID, StagerConstants::kMotorType},
                                     m_stagerFollower{StagerConstants::kFollowerID, StagerConstants::kMotorType}{
  // Implementation of subsystem constructor goes here.

  #ifdef BURNSTAGERSPARKMAX
  // Restore to deafaults
  m_stagerMotor.RestoreFactoryDefaults();
  m_stagerFollower.RestoreFactoryDefaults();

  m_stagerMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  m_stagerFollower.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  
  // Enable voltage Compensation
  m_stagerMotor.EnableVoltageCompensation(RobotConstants::kVoltageCompentationValue);
  m_stagerFollower.EnableVoltageCompensation(RobotConstants::kVoltageCompentationValue);

  // Set follwers and inversted state
  m_stagerFollower.Follow(m_stagerMotor, true);

  m_stagerMotor.BurnFlash();
  m_stagerFollower.BurnFlash();

  printf("Flash Burned on shooter subsystem\r\n");
  #else
  printf("Flash was not burned on shooter subsystem\r\n");
  #endif

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("Stager");

  nte_color = nt_table->GetEntry("Note Loaded");
  nte_colorBlue = nt_table->GetEntry("Color/Blue");
  nte_colorRed = nt_table->GetEntry("Color/Red");
  nte_colorGreen = nt_table->GetEntry("Color/Green");
  nte_proximity = nt_table->GetEntry("Color/Proximity");
  
}

void StagerSubsystem::Periodic() {
  nte_color.SetBoolean(NoteLoaded());
  nte_colorBlue.SetDouble((double)m_colorSensor.GetColor().blue);
  nte_colorRed.SetDouble((double)m_colorSensor.GetColor().red);
  nte_colorGreen.SetDouble((double)m_colorSensor.GetColor().green);
  nte_proximity.SetDouble((double)m_colorSensor.GetProximity());
}

void StagerSubsystem::SetMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_stagerMotor.Set(power);
}

bool StagerSubsystem::NoteLoaded() {
  if ((double)m_colorSensor.GetProximity() > 500.0) // Erik put 500.0
    return true;
  else 
    return false;
}