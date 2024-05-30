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

class GuftSubsystem : public frc2::SubsystemBase {
 public:

 /**
  * This is the Guft subsystem (guarenteeification subsystem) it holds the arm that can be deployed that guarentees our amp shot
  * 
  * @param GuftAngleOffset // Angle offset for the guft encoder
 */
  GuftSubsystem(double GuftAngleOffset);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * DO NOT SET FULL SPEED TO THIS
  */
  void SetMotorPower(double power); 

  /**
   * Sets the Guft angle using closed loop control on the SparkMax
   * 
   * @param angle The desired angle in radians
  */
  void SetGuftAngle(units::radian_t angle);

    /**
   * Sets the Guft angle using closed loop control on the SparkMax directly to the preset deployment/retraction angles
   * 
   * @param isDeployed The desired angle in radians
  */
  void SetGuftDeployed(bool isDeployed);

  /**
   * Gets the angle of Guft with the offset accounted for
   * 
   * @return The angle of the Guft in radians
  */
  double GetGuftAngle();
  
  /**
   * Gets if the PID Controller is at the setpoint
   * 
   * @return True if at the Angle setpoint
  */
  bool AtAngleSetpoint();

 private:
  // Local copy of arguments
  double m_guftAngleOffset = 0.0;

  // Networktable entries

  nt::NetworkTableEntry nte_guftAngle;
  nt::NetworkTableEntry nte_guftSetpoint;

  // Motor Controllers;
  rev::CANSparkMax m_guftMotor;

  // Encoders motor controllers
  rev::SparkAbsoluteEncoder m_guftAbsoluteEncoder = m_guftMotor.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

  // PID Contollers for
  rev::SparkPIDController m_guftPIDController = m_guftMotor.GetPIDController();
};