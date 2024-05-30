// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GuftSubsystem.h"
#include <networktables/NetworkTableInstance.h>

using namespace GuftConstants;

GuftSubsystem::GuftSubsystem(double guftAngleOffset) : m_guftMotor{kGuftID, kGuftMotorType} {
  
  // Burn flash only if desired - true set in constants
  #ifdef BURNGUFTSPARKMAX 
  // Restore deafults
  m_guftMotor.RestoreFactoryDefaults();
  
  // Enable Voltage
  m_guftMotor.EnableVoltageCompensation(RobotConstants::kVoltageCompentationValue);
  
  // Set converstion factors for encoders
  m_guftAbsoluteEncoder.SetPositionConversionFactor(kGuftPositionFactor);
  m_guftAbsoluteEncoder.SetVelocityConversionFactor(kGuftEncoderVelocityFactor);
  m_guftAbsoluteEncoder.SetZeroOffset(guftAngleOffset);
  
  // Set PID Constants
  m_guftPIDController.SetP(kGuftP);
  m_guftPIDController.SetI(kGuftI);
  m_guftPIDController.SetD(kGuftD);
  m_guftPIDController.SetFF(kGuftFF);
  m_guftPIDController.SetOutputRange(kGuftMin, kGuftMax);

  // Set Idle mode (what to do when not commanded at a speed)
  m_guftMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  m_guftMotor.SetSmartCurrentLimit(kGuftMotorCurrentLimit.value());

 

  // Set the PID Controller to use the duty cycle encoder on the swerve
  // module instead of the built in NEO550 encoder.
  m_guftPIDController.SetFeedbackDevice(m_guftAbsoluteEncoder);
  m_guftMotor.SetInverted(true);

  m_guftMotor.BurnFlash();

  std::cout << "Flash Burned on Guft subsystem\r\n";
  #else
  std::cout << "Flash was not burned on Guft subsystem\r\n";
  #endif

  m_guftAngleOffset = guftAngleOffset;

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("Guft");  

  nte_guftAngle = nt_table->GetEntry("Guft Angle");
  nte_guftSetpoint = nt_table->GetEntry("Guft Setpoint");

}

void GuftSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  //nte_guftAngle.SetDouble((double)m_guftAbsoluteEncoder.GetPosition());
}

double GuftSubsystem::GetGuftAngle() {
  return m_guftAbsoluteEncoder.GetPosition();
}

void GuftSubsystem::SetGuftAngle(units::radian_t angle) {
  // Set the setpoint as the input angle
  if ((double)angle > kMaxGuftAngle)
    angle = (units::radian_t)kMaxGuftAngle;
  if ((double)angle < kMinGuftAngle)
    angle = (units::radian_t)kMinGuftAngle;
  m_guftPIDController.SetReference((double)angle, rev::CANSparkMax::ControlType::kPosition);
  nte_guftSetpoint.SetDouble((double)angle);
}

bool GuftSubsystem::AtAngleSetpoint() {
  if (abs(nte_guftSetpoint.GetDouble(1.0) - m_guftAbsoluteEncoder.GetPosition()) < 0.05) // in radians
    return true;
  else
    return false;
}