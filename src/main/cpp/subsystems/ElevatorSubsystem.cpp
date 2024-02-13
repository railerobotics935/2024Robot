// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"
#include "Constants.h"
#include <networktables/NetworkTableInstance.h>

using namespace ElevatorConstants;

ElevatorSubsystem::ElevatorSubsystem() : m_elevatorMotor{kElevatorID, kElevatorMotorType},
                                         m_followerMotor{kFollowerID, kFollowerMotorType} {

  // Burn flash only if desired - true set in constants
  #ifdef BURNSPARKMAX 
  // Restore deafults
  m_elevatorMotor.RestoreFactoryDefaults();
  m_followerMotor.RestoreFactoryDefaults();

  // Set converstion factors for encoders
  m_elevatorEncoder.SetPositionConversionFactor(kElevatorPositionFactor);
  m_elevatorEncoder.SetVelocityConversionFactor(kElevatorEncoderVelocityFactor);

  // Set PID Constants
  m_elevatorPIDController.SetP(kElevatorP);
  m_elevatorPIDController.SetI(kElevatorI);
  m_elevatorPIDController.SetD(kElevatorD);
  m_elevatorPIDController.SetFF(kElevatorFF);
  m_elevatorPIDController.SetOutputRange(kElevatorMin, kElevatorMax);

  // Set Idle mode (what to do when not commanded at a speed)
  m_elevatorMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  m_followerMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  m_elevatorMotor.SetSmartCurrentLimit(kElevatorMotorCurrentLimit.value());
  m_followerMotor.SetSmartCurrentLimit(kFollowerMotorCurrentLimit.value());

  m_elevatorMotor.BurnFlash();
  m_followerMotor.BurnFlash();
  printf("Flash Burned on elevator subsystem\r\n");
  #else
  printf("Flash was not burned on elevator subsystem\r\n");
  #endif

  // Set folower motor
  m_followerMotor.Follow(m_elevatorMotor, false); // check inversion for follower

   // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("Elevator");  

  nte_elevatorPosition = nt_table->GetEntry("Elevator Position");
  nte_elevatorVelocity = nt_table->GetEntry("Elevator Velocity");
  nte_elevatorSetpoint = nt_table->GetEntry("Elevator Setpoint");
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
  nte_elevatorPosition.SetDouble(m_elevatorEncoder.GetPosition());
  nte_elevatorVelocity.SetDouble(m_elevatorEncoder.GetVelocity());
}

void ElevatorSubsystem::SetElevatorPosition(units::meter_t extentionDistance) {
  // Set setpoint for PID controller
  m_elevatorPIDController.SetReference((double)extentionDistance, rev::CANSparkMax::ControlType::kPosition);
  nte_elevatorSetpoint.SetDouble((double)extentionDistance);
}

