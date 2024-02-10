// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"
#include <networktables/NetworkTableInstance.h>

using namespace ShooterConstants;

ShooterSubsystem::ShooterSubsystem() : m_shooterMotor{kShooterID, kShooterMotorType},
                    m_followerMotor{kFollowerID, kFollowerMotorType},
                    m_pitchMotor{kPitchID, kPitchMotorType}{
                      
  // Burn flash only if desired - true set in constants
  #ifdef BURNSPARKMAX 
  // Restore deafults
  m_shooterMotor.RestoreFactoryDefaults();
  m_followerMotor.RestoreFactoryDefaults();
  m_pitchMotor.RestoreFactoryDefaults();

  // Set converstion factors for encoders
  m_shooterEncoder.SetPositionConversionFactor(kShooterPositionFactor);
  m_shooterEncoder.SetVelocityConversionFactor(kShooterEncoderVelocityFactor);
  m_pitchAbsoluteEncoder.SetPositionConversionFactor(kPitchPositionFactor);
  m_pitchAbsoluteEncoder.SetVelocityConversionFactor(kPitchEncoderVelocityFactor);

  // Set the PID Controller to use the duty cycle encoder on the swerve
  // module instead of the built in NEO550 encoder.
  m_pitchPIDController.SetFeedbackDevice(m_pitchAbsoluteEncoder);

  // Set PID Constants
  m_shooterPIDController.SetP(kShooterP);
  m_shooterPIDController.SetI(kShooterI);
  m_shooterPIDController.SetD(kShooterD);
  m_shooterPIDController.SetFF(kShooterFF);
  m_shooterPIDController.SetOutputRange(kShooterMin, kShooterMax);

  // Set PID Constants
  m_pitchPIDController.SetP(kPitchP);
  m_pitchPIDController.SetI(kPitchI);
  m_pitchPIDController.SetD(kPitchD);
  m_pitchPIDController.SetFF(kPitchFF);
  m_pitchPIDController.SetOutputRange(kPitchMin, kPitchMax);

  // Set Idle mode (what to do when not commanded at a speed)
  m_shooterMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  m_followerMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  m_pitchMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  m_shooterMotor.SetSmartCurrentLimit(kShooterMotorCurrentLimit.value());
  m_followerMotor.SetSmartCurrentLimit(kFollowerMotorCurrentLimit.value());
  m_pitchMotor.SetSmartCurrentLimit(kPitchMotorCurrentLimit.value());

  m_shooterMotor.BurnFlash();
  m_followerMotor.BurnFlash();
  m_pitchMotor.BurnFlash();
  printf("Flash Burned on shooter subsystem\r\n");
  #else
  printf("Flash was not burned on shooter subsystem\r\n");
  #endif

  // Set folower motor
  m_followerMotor.Follow(m_shooterMotor, true);

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("Shooter");  

  nte_shooterSpeed = nt_table->GetEntry("Shooter Speed");
  nte_followerSpeed = nt_table->GetEntry("Follower Speed");
  nte_shooterSetpoint = nt_table->GetEntry("Shooter Setpoint");
  nte_pitchAngle = nt_table->GetEntry("Pitch Angle");
  nte_pitchSetpoint = nt_table->GetEntry("Pitch Setpoint");
}

void ShooterSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  nte_shooterSpeed.SetDouble((double)m_shooterEncoder.GetVelocity());
  nte_followerSpeed.SetDouble((double)m_followerEncoder.GetVelocity());
  nte_shooterSetpoint.SetDouble(m_shooterSetpoint);
  nte_pitchAngle.SetDouble((double)m_pitchAbsoluteEncoder.GetPosition());
  nte_pitchSetpoint.SetDouble(m_pitchSetpoint);
}

void ShooterSubsystem::SetMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_shooterMotor.Set(power);
}

void ShooterSubsystem::SetShooterAngle(units::radian_t angle) {
  // Set the setpoint as the input angle
  m_pitchPIDController.SetReference((double)angle, rev::CANSparkMax::ControlType::kPosition);
  m_pitchSetpoint = (double)angle;
}

void ShooterSubsystem::SetShooterSpeed(units::radians_per_second_t speed) {
  // Set the setpoint as the input angle
  m_shooterPIDController.SetReference((double)speed, rev::CANSparkMax::ControlType::kVelocity);
  m_shooterSetpoint = (double)speed;
}