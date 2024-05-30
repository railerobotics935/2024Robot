// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GuftSubsystem.h"
#include <networktables/NetworkTableInstance.h>

using namespace GuftConstants;

GuftSubsystem::GuftSubsystem(double GuftAngleOffset) : m_topGuftMotor{kTopGuftID, kGuftMotorType},
                    m_bottomGuftMotor{kBottomGuftID, kGuftMotorType},
                    m_pitchMotor{kPitchID, kPitchMotorType} {
  
  // Burn flash only if desired - true set in constants
  #ifdef BURNGuftSPARKMAX 
  // Restore deafults
  m_topGuftMotor.RestoreFactoryDefaults();
  m_bottomGuftMotor.RestoreFactoryDefaults();
  m_pitchMotor.RestoreFactoryDefaults();
  
  // Enable Voltage
  m_topGuftMotor.EnableVoltageCompensation(RobotConstants::kVoltageCompentationValue);
  m_bottomGuftMotor.EnableVoltageCompensation(RobotConstants::kVoltageCompentationValue);
  m_pitchMotor.EnableVoltageCompensation(RobotConstants::kVoltageCompentationValue);
  
  // Set converstion factors for encoders
  m_topGuftEncoder.SetPositionConversionFactor(kGuftPositionFactor);
  m_topGuftEncoder.SetVelocityConversionFactor(kGuftEncoderVelocityFactor);
  m_bottomGuftEncoder.SetPositionConversionFactor(kGuftPositionFactor);
  m_bottomGuftEncoder.SetVelocityConversionFactor(kGuftEncoderVelocityFactor);
  m_pitchAbsoluteEncoder.SetPositionConversionFactor(kPitchPositionFactor);
  m_pitchAbsoluteEncoder.SetVelocityConversionFactor(kPitchEncoderVelocityFactor);
  m_pitchAbsoluteEncoder.SetZeroOffset(GuftAngleOffset);
  
 // m_GuftPIDController.SetFeedbackDevice(m_GuftEncoder);

  // Set PID Constants
  m_topGuftPIDController.SetP(kTopGuftP);
  m_topGuftPIDController.SetI(kTopGuftI);
  m_topGuftPIDController.SetD(kTopGuftD);
  m_topGuftPIDController.SetFF(kTopGuftFF);
  m_topGuftPIDController.SetOutputRange(kTopGuftMin, kTopGuftMax);

  m_bottomGuftPIDController.SetP(kBottomGuftP);
  m_bottomGuftPIDController.SetI(kBottomGuftI);
  m_bottomGuftPIDController.SetD(kBottomGuftD);
  m_bottomGuftPIDController.SetFF(kBottomGuftFF);
  m_bottomGuftPIDController.SetOutputRange(kBottomGuftMin, kBottomGuftMax);

  // Set PID Constants
  m_pitchPIDController.SetP(kPitchP);
  m_pitchPIDController.SetI(kPitchI);
  m_pitchPIDController.SetD(kPitchD);
  m_pitchPIDController.SetFF(kPitchFF);
  m_pitchPIDController.SetOutputRange(kPitchMin, kPitchMax);

  // Set Idle mode (what to do when not commanded at a speed)
  m_topGuftMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  m_bottomGuftMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  m_pitchMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  m_topGuftMotor.SetSmartCurrentLimit(kGuftMotorCurrentLimit.value());
  m_bottomGuftMotor.SetSmartCurrentLimit(kGuftMotorCurrentLimit.value());
  m_pitchMotor.SetSmartCurrentLimit(kPitchMotorCurrentLimit.value());

  m_pitchMotor.SetInverted(true);

  // Set the PID Controller to use the duty cycle encoder on the swerve
  // module instead of the built in NEO550 encoder.
  m_pitchPIDController.SetFeedbackDevice(m_pitchAbsoluteEncoder);
  m_pitchMotor.SetInverted(true);

  // Set the PID Controller to use the duty cycle encoder on the swerve
  // module instead of the built in NEO550 encoder.
  m_pitchPIDController.SetFeedbackDevice(m_pitchAbsoluteEncoder);

  m_bottomGuftMotor.SetInverted(true);

  m_topGuftMotor.BurnFlash();
  m_bottomGuftMotor.BurnFlash();
  m_pitchMotor.BurnFlash();

  std::cout << "Flash Burned on Guft subsystem\r\n";
  #else
  std::cout << "Flash was not burned on Guft subsystem\r\n";
  #endif

  m_GuftAngleOffset = GuftAngleOffset;

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("Guft");  

  nte_topGuftSpeed = nt_table->GetEntry("Top Guft Speed");
  nte_bottomGuftSpeed = nt_table->GetEntry("Bottom Guft Speed");
  nte_topGuftSetpoint = nt_table->GetEntry("Top Guft Setpoint");
  nte_bottomGuftSetpoint = nt_table->GetEntry("Bottom Guft Setpoint");
  nte_pitchAngle = nt_table->GetEntry("Pitch Angle");
  nte_pitchSetpoint = nt_table->GetEntry("Pitch Setpoint");
  nte_topSetpointSpeedRPM = nt_table->GetEntry("Top Setpoint Speed in RPM");
  nte_bottomSetpointSpeedRPM = nt_table->GetEntry("Bottom Setpoint Speed in RPM");
  nte_setpointAngleRadians = nt_table->GetEntry("Setpoint Angle in Radians");

  nte_topSetpointSpeedRPM.SetDouble(0.0);
  nte_setpointAngleRadians.SetDouble(0.8);

}

void GuftSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  //nte_topGuftSpeed.SetDouble((double)m_topGuftEncoder.GetVelocity());
  //nte_bottomGuftSpeed.SetDouble((double)m_bottomGuftEncoder.GetVelocity());
  //nte_pitchAngle.SetDouble((double)m_pitchAbsoluteEncoder.GetPosition());
}

double GuftSubsystem::GetGuftAngle() {
  return m_pitchAbsoluteEncoder.GetPosition();
}

void GuftSubsystem::SetGuftAngle(units::radian_t angle) {
  // Set the setpoint as the input angle
  if ((double)angle > kMaxPitchAngle)
    angle = (units::radian_t)kMaxPitchAngle;
  if ((double)angle < kMinPitchAngle)
    angle = (units::radian_t)kMinPitchAngle;
  m_pitchPIDController.SetReference((double)angle, rev::CANSparkMax::ControlType::kPosition);
  nte_pitchSetpoint.SetDouble((double)angle);
}

bool GuftSubsystem::AtAngleSetpoint() {
  if (abs(nte_pitchSetpoint.GetDouble(1.0) - m_pitchAbsoluteEncoder.GetPosition()) < 0.05) // in radians
    return true;
  else
    return false;
}