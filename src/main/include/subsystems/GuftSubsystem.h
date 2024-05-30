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
  enum GuftState {Retracted, Deployed}; 


 /**
  * 
  * 
  * @param GuftAngleOffset // Angle offset for the pitch encoder
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
  double m_GuftAngleOffset = 0.0;

  // Networktable entries
  nt::NetworkTableEntry nte_topGuftSpeed;
  nt::NetworkTableEntry nte_bottomGuftSpeed;
  nt::NetworkTableEntry nte_topGuftSetpoint;
  nt::NetworkTableEntry nte_bottomGuftSetpoint;
  nt::NetworkTableEntry nte_pitchAngle;
  nt::NetworkTableEntry nte_pitchSetpoint;
  nt::NetworkTableEntry nte_topSetpointSpeedRPM;
  nt::NetworkTableEntry nte_bottomSetpointSpeedRPM;
  nt::NetworkTableEntry nte_setpointAngleRadians;

  // Motor Controllers
  rev::CANSparkMax m_topGuftMotor;
  rev::CANSparkMax m_bottomGuftMotor;
  rev::CANSparkMax m_pitchMotor;

  // Encoders motor controllers
  rev::SparkRelativeEncoder m_topGuftEncoder = m_topGuftMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder m_bottomGuftEncoder = m_bottomGuftMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkAbsoluteEncoder m_pitchAbsoluteEncoder = m_pitchMotor.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

  // PID Contollers for
  rev::SparkPIDController m_topGuftPIDController = m_topGuftMotor.GetPIDController();
  rev::SparkPIDController m_bottomGuftPIDController = m_bottomGuftMotor.GetPIDController();
  rev::SparkPIDController m_pitchPIDController = m_pitchMotor.GetPIDController();
};