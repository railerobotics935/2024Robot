// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/DigitalInput.h>
#include <frc/SensorUtil.h>

#include "Constants.h"

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * Creates a Climber subsystem.
   * Currently for both indiviual climbers (two phystical subsystems)
   * but coding it as one
  */
  ClimberSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override; 

  /**
   * @returns True if the left climber limit switch is pressed
  */
  bool LeftClimberAtBase();

  /**
   * @returns True if the right climber limit switch is pressed
  */
  bool RightClimberAtBase();

  /**
   * Set the climber moter to a power
   * 
   * @param power Power to set the motor power
  */
  void SetClimberPower(double power);

  /**
   * Set the climber motor power invidualy
  */
  void SetIndividualClimberPower(double power);

 private:

  nt::NetworkTableEntry m_leftCllimberLimitSwtich;
  nt::NetworkTableEntry m_leftClimberDistance;
  nt::NetworkTableEntry m_rightCllimberLimitSwtich;
  nt::NetworkTableEntry m_rightClimberDistance;

  // Motor Controllers
  rev::CANSparkMax m_leftClimberMotor{ClimberConstants::LeftClimber::kID, ClimberConstants::kMotorType};
  rev::CANSparkMax m_rightClimberMotor{ClimberConstants::RightClimber::kID, ClimberConstants::kMotorType};
  
  // Encoders motor controllers
  rev::SparkRelativeEncoder m_leftClimberEncoder = m_leftClimberMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder m_rightClimberEncoder = m_rightClimberMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  // Limit switch is a digital input in the DIO port (digital input output)
  frc::DigitalInput m_leftLimitSwitch{ClimberConstants::LeftClimber::kLimitSwitchPort};
  frc::DigitalInput m_rightLimitSwitch{ClimberConstants::RightClimber::kLimitSwitchPort};
};