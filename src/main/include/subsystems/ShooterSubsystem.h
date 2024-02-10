// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

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

  //Sets the motor's power (between -1.0 and 1.0).
  void SetMotorPower(double power);

  void AimAtTarget();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Motor Controllers
  rev::CANSparkMax m_shooterMotor;
  rev::CANSparkMax m_shooterFollower;
  rev::CANSparkMax m_pitchMotor;
  
};