// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/ColorSensorV3.h>
#include <rev/CANSparkMax.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * Picks up game pieces
  */
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Sets the motor's power (between -1.0 and 1.0).
  void SetMotorPower(double power);

  /**
   * If it sees a note, it will return true
   * 
   * @return true if a note is over the sensor
  */
  bool NoteLoaded();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::ColorSensorV3 m_colorSensor{frc::I2C::Port::kOnboard};

  // Motor Controllers
  rev::CANSparkMax m_intakeMotor;
};
