
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "subsystems/ShooterSubsystem.h"

/**
 * Command to take shooter speed and angle from nte and set the robot to shoot it
 */
class ManualShoot 
  : public frc2::CommandHelper<frc2::Command, ManualShoot> {
public:
  /**
   * Sets shooter to values in networktable entries
   * 
   * @param shooter memory adress of shooter subsystem
   * @param opController Memory adress of operator controller
   * @param pitchAngle Angle in radians the shooter should be set to
   * @param topShooterSpeed Speed in RPM the top shooter should be set to
   * @param bottomShooterSpeed Sepped in RPM the bottom shooter should be set to
  */
  explicit ManualShoot(ShooterSubsystem* shooter, 
                       frc::XboxController* opController, 
                       units::radian_t pitchAngle, 
                       units::revolutions_per_minute_t topShooterSpeed, 
                       units::revolutions_per_minute_t bottomShooterSpeed);

  void Initialize() override;
  void End(bool interrupted) override;

private:
// Declare local subsystems
ShooterSubsystem* m_shooter;
frc::XboxController* m_opController;

// Shooting values
units::radian_t m_pitchAngle; 
units::revolutions_per_minute_t m_topShooterSpeed;
units::revolutions_per_minute_t m_bottomShooterSpeed;

// Declare network table entries
nt::NetworkTableEntry nte_setpointSpeedRPM;
nt::NetworkTableEntry nte_setpointAngleRadians;
};