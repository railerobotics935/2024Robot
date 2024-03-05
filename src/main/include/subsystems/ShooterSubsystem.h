// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <networktables/NetworkTableEntry.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>               // for through-hole encoder on Spark MAX
#include <rev/SparkPIDController.h>                 // for closed-loop control on Spark MAX 
#include <rev/SparkRelativeEncoder.h>               // for closed-loop control on Spark MAX
#include <units/angle.h>
#include <frc/controller/PIDController.h>
#include "Constants.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
 /**
  * 
  * 
  * @param shooterAngleOffset // Angle offset for the pitch encoder
 */
  ShooterSubsystem(double shooterAngleOffset);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  //Sets the motor's power (between -1.0 and 1.0).
  void SetShooterMotorPower(double power);

  /**
   * DO NOT SET FULL SPEED TO THIS
  */
  void SetPitchMotorPower(double power); 
  /**
   * Sets the Shooter angle using closed loop control on the SparkMax
   * 
   * @param angle The desired angle in radians
  */
  void SetShooterAngle(units::radian_t angle);

  /**
   * Sets the Shooter speed using closed loop control on the SparkMax
   * 
   * @param speed The desired speed in radians per second
  */
  void SetShooterSpeed(units::revolutions_per_minute_t speed);

  /**
   * Gets the angle of shooter with the offset accounted for
   * 
   * @return The angle of the shooter in radians
  */
  double GetShooterAngle();
  
  /**
   * Takes values from NT to set speed and position of the shooter
  */
  void ManualNteShoot();

  /**
   * Gets if the PID Controller is at the setpoint
   * 
   * @return True if at the Angle setpoint
  */
  bool AtAngleSetpoint();

  /**
   * Gets if the PID Controller is at the setpoint
   * 
   * @return True if at the Speed setpoint
  */
  bool AtSpeedSetpoint();

 private:
  // Local copy of arguments
  double m_shooterAngleOffset = 0.0;

  // Networktable entries
  nt::NetworkTableEntry nte_shooterSpeed;
  nt::NetworkTableEntry nte_followerSpeed;
  nt::NetworkTableEntry nte_shooterSetpoint;
  nt::NetworkTableEntry nte_pitchAngle;
  nt::NetworkTableEntry nte_pitchSetpoint;
  nt::NetworkTableEntry nte_setpointSpeedRPM;
  nt::NetworkTableEntry nte_setpointAngleRadians;

  // Motor Controllers
  rev::CANSparkMax m_shooterMotor;
  rev::CANSparkMax m_followerMotor;
  rev::CANSparkMax m_pitchMotor;

  // Encoders motor controllers
  rev::SparkRelativeEncoder m_shooterEncoder = m_shooterMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder m_followerEncoder = m_followerMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkAbsoluteEncoder m_pitchAbsoluteEncoder = m_pitchMotor.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

  // PID Contollers for
  rev::SparkPIDController m_shooterPIDController = m_shooterMotor.GetPIDController();
  rev::SparkPIDController m_pitchPIDController = m_pitchMotor.GetPIDController();
};