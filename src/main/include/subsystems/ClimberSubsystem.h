// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/DigitalInput.h>

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * Creates a climber subsytem
   * ONLY CREATES ONE
   * 
   * @param motorId CAN Id of the motor
   * @param LimitSwitchPort Port number of the limit switch
  */
  ClimberSubsystem(int motorId, int LimitSwitchPort, bool reversed);


  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool ExampleCondition();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Set the climber moter to a power
   * 
   * @param power Power to set the motor power
  */
  void SetClimberPower(double power);


 private:

  // Motor Controllers
  rev::CANSparkMax m_climberMotor;
  
  // Encoders motor controllers
  rev::SparkRelativeEncoder m_climberEncoder = m_climberMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  // Limit switch is a digital input in the DIO port (digital input output)
  frc::DigitalInput m_limitSwitch;
};