// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

class StagerSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * Controls movement of game piece between intake and shooter
  */
  StagerSubsystem();

  // Sets the motor's power (between -1.0 and 1.0).
  void SetMotorPower(double power);

 private:
  // Motor Controllers
  rev::CANSparkMax m_stagerMotor;
  rev::CANSparkMax m_stagerFollower;
};
