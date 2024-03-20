// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"
#include <networktables/NetworkTableInstance.h>

using namespace ShooterConstants;

ShooterSubsystem::ShooterSubsystem(double shooterAngleOffset) : m_topShooterMotor{kTopShooterID, kShooterMotorType},
                    m_bottomShooterMotor{kBottomShooterID, kShooterMotorType},
                    m_pitchMotor{kPitchID, kPitchMotorType} {
  
  // Burn flash only if desired - true set in constants
  #ifdef BURNSHOOTERSPARKMAX 
  // Restore deafults
  m_topShooterMotor.RestoreFactoryDefaults();
  m_bottomShooterMotor.RestoreFactoryDefaults();
  m_pitchMotor.RestoreFactoryDefaults();
  
  // Enable Voltage
  m_topShooterMotor.EnableVoltageCompensation(RobotConstants::kVoltageCompentationValue);
  m_bottomShooterMotor.EnableVoltageCompensation(RobotConstants::kVoltageCompentationValue);
  m_pitchMotor.EnableVoltageCompensation(RobotConstants::kVoltageCompentationValue);
  
  // Set converstion factors for encoders
  m_topShooterEncoder.SetPositionConversionFactor(kShooterPositionFactor);
  m_topShooterEncoder.SetVelocityConversionFactor(kShooterEncoderVelocityFactor);
  m_bottomShooterEncoder.SetPositionConversionFactor(kShooterPositionFactor);
  m_bottomShooterEncoder.SetVelocityConversionFactor(kShooterEncoderVelocityFactor);
  m_pitchAbsoluteEncoder.SetPositionConversionFactor(kPitchPositionFactor);
  m_pitchAbsoluteEncoder.SetVelocityConversionFactor(kPitchEncoderVelocityFactor);
  m_pitchAbsoluteEncoder.SetZeroOffset(shooterAngleOffset);
  
 // m_shooterPIDController.SetFeedbackDevice(m_shooterEncoder);

  // Set PID Constants
  m_topShooterPIDController.SetP(kTopShooterP);
  m_topShooterPIDController.SetI(kTopShooterI);
  m_topShooterPIDController.SetD(kTopShooterD);
  m_topShooterPIDController.SetFF(kTopShooterFF);
  m_topShooterPIDController.SetOutputRange(kTopShooterMin, kTopShooterMax);

  m_bottomShooterPIDController.SetP(kBottomShooterP);
  m_bottomShooterPIDController.SetI(kBottomShooterI);
  m_bottomShooterPIDController.SetD(kBottomShooterD);
  m_bottomShooterPIDController.SetFF(kBottomShooterFF);
  m_bottomShooterPIDController.SetOutputRange(kBottomShooterMin, kBottomShooterMax);

  // Set PID Constants
  m_pitchPIDController.SetP(kPitchP);
  m_pitchPIDController.SetI(kPitchI);
  m_pitchPIDController.SetD(kPitchD);
  m_pitchPIDController.SetFF(kPitchFF);
  m_pitchPIDController.SetOutputRange(kPitchMin, kPitchMax);

  // Set Idle mode (what to do when not commanded at a speed)
  m_topShooterMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  m_bottomShooterMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  m_pitchMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  m_topShooterMotor.SetSmartCurrentLimit(kShooterMotorCurrentLimit.value());
  m_bottomShooterMotor.SetSmartCurrentLimit(kShooterMotorCurrentLimit.value());
  m_pitchMotor.SetSmartCurrentLimit(kPitchMotorCurrentLimit.value());

  m_pitchMotor.SetInverted(true);

  // Set the PID Controller to use the duty cycle encoder on the swerve
  // module instead of the built in NEO550 encoder.
  m_pitchPIDController.SetFeedbackDevice(m_pitchAbsoluteEncoder);
  m_pitchMotor.SetInverted(true);

  // Set the PID Controller to use the duty cycle encoder on the swerve
  // module instead of the built in NEO550 encoder.
  m_pitchPIDController.SetFeedbackDevice(m_pitchAbsoluteEncoder);

  m_bottomShooterMotor.SetInverted(true);

  m_topShooterMotor.BurnFlash();
  m_bottomShooterMotor.BurnFlash();
  m_pitchMotor.BurnFlash();

  std::cout << "Flash Burned on shooter subsystem\r\n";
  #else
  std::cout << "Flash was not burned on shooter subsystem\r\n";
  #endif

  m_shooterAngleOffset = shooterAngleOffset;

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("Shooter");  

  nte_topShooterSpeed = nt_table->GetEntry("Top Shooter Speed");
  nte_bottomShooterSpeed = nt_table->GetEntry("Bottom Shooter Speed");
  nte_topShooterSetpoint = nt_table->GetEntry("Top Shooter Setpoint");
  nte_bottomShooterSetpoint = nt_table->GetEntry("Bottom Shooter Setpoint");
  nte_pitchAngle = nt_table->GetEntry("Pitch Angle");
  nte_pitchSetpoint = nt_table->GetEntry("Pitch Setpoint");
  nte_topSetpointSpeedRPM = nt_table->GetEntry("Top Setpoint Speed in RPM");
  nte_bottomSetpointSpeedRPM = nt_table->GetEntry("Bottom Setpoint Speed in RPM");
  nte_setpointAngleRadians = nt_table->GetEntry("Setpoint Angle in Radians");

  nte_topSetpointSpeedRPM.SetDouble(0.0);
  nte_setpointAngleRadians.SetDouble(0.8);

}

void ShooterSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  nte_topShooterSpeed.SetDouble((double)m_topShooterEncoder.GetVelocity());
  nte_bottomShooterSpeed.SetDouble((double)m_bottomShooterEncoder.GetVelocity());
  nte_pitchAngle.SetDouble((double)m_pitchAbsoluteEncoder.GetPosition());
}

void ShooterSubsystem::SetShooterMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  m_topShooterMotor.Set(power);
  m_bottomShooterMotor.Set(power);
}

void ShooterSubsystem::SetPitchMotorPower(double power) {
  // Sets the motor's power (between -1.0 and 1.0). 
  if (m_pitchAbsoluteEncoder.GetPosition() > kMaxPitchAngle && power > 0.0)
    m_pitchMotor.Set(0.0);
  else if (m_pitchAbsoluteEncoder.GetPosition() < kMinPitchAngle && power < 0.0)
    m_pitchMotor.Set(0.0);
  else
    m_pitchMotor.Set(power);
}

void ShooterSubsystem::ManualNteShoot() {
  // Set shooter to angle and speed from shuffleboard
  SetShooterAngle((units::radian_t)nte_setpointAngleRadians.GetDouble(1.0));
  SetIndividualShooterSpeed((units::revolutions_per_minute_t)nte_topSetpointSpeedRPM.GetDouble(0.0), (units::revolutions_per_minute_t)nte_bottomShooterSetpoint.GetDouble(0.0));
}

void ShooterSubsystem::SetShooterAngle(units::radian_t angle) {
  // Set the setpoint as the input angle
  if ((double)angle > kMaxPitchAngle)
    angle = (units::radian_t)kMaxPitchAngle;
  if ((double)angle < kMinPitchAngle)
    angle = (units::radian_t)kMinPitchAngle;
  m_pitchPIDController.SetReference((double)angle, rev::CANSparkMax::ControlType::kPosition);
  nte_pitchSetpoint.SetDouble((double)angle);
}

void ShooterSubsystem::SetShooterSpeed(units::revolutions_per_minute_t speed) {
  // Set the setpoint as the input angle
  m_topShooterPIDController.SetReference((double)speed, rev::CANSparkMax::ControlType::kVelocity);
  m_bottomShooterPIDController.SetReference((double)speed, rev::CANSparkMax::ControlType::kVelocity);
  nte_topShooterSetpoint.SetDouble((double)speed);
}

void ShooterSubsystem::SetIndividualShooterSpeed(units::revolutions_per_minute_t topSpeed, units::revolutions_per_minute_t bottomSpeed) {
  // Set the setpoint as the input angle
  m_topShooterPIDController.SetReference((double)topSpeed, rev::CANSparkMax::ControlType::kVelocity);
  m_bottomShooterPIDController.SetReference((double)bottomSpeed, rev::CANSparkMax::ControlType::kVelocity);
  nte_topShooterSetpoint.SetDouble((double)topSpeed);
  nte_bottomShooterSetpoint.SetDouble((double)bottomSpeed);
}

bool ShooterSubsystem::AtAngleSetpoint() {
  if (abs(nte_pitchSetpoint.GetDouble(1.0) - m_pitchAbsoluteEncoder.GetPosition()) < 0.05) // in radians
    return true;
  else
    return false;
}

bool ShooterSubsystem::AtSpeedSetpoint() {
  if (abs(nte_topShooterSetpoint.GetDouble(0.0) - m_topShooterEncoder.GetVelocity()) < 10.0 &&
      abs(nte_bottomShooterSetpoint.GetDouble(0.0) - m_bottomShooterEncoder.GetVelocity()) < 10.0) // in RPM
    return true;
  else
    return false;
}