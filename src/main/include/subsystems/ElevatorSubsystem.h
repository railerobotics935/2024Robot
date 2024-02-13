// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();

  /**
   * Example command factory method.
   */
  frc2::CommandPtr ExampleMethodCommand();

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

 private:

  // Motor Controllers
  rev::CANSparkMax m_elevatorMotor;
  rev::CANSparkMax m_followerMotor;
  
  // Encoders motor controllers
  rev::SparkRelativeEncoder m_elevatorEncoder = m_elevatorMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder m_followerEncoder = m_followerMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  // PID Contollers for
  rev::SparkPIDController m_elevatorPIDController = m_elevatorMotor.GetPIDController();
};