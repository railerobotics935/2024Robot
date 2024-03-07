// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/ColorSensorV3.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>


class StagerSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * Controls movement of game piece between intake and shooter
  */
  StagerSubsystem();

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
  nt::NetworkTableEntry nte_color;
  nt::NetworkTableEntry nte_colorBlue;
  nt::NetworkTableEntry nte_colorRed;
  nt::NetworkTableEntry nte_colorGreen;
  nt::NetworkTableEntry nte_proximity;  

  // Motor Controllers
  rev::CANSparkMax m_stagerMotor;
  rev::CANSparkMax m_stagerFollower;

  //frc::Color noteColor{0.0152, 0.401, 0.444};

  rev::ColorSensorV3 m_colorSensor{frc::I2C::Port::kOnboard};
};
