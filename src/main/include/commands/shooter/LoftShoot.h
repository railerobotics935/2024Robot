
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
class LoftShoot 
  : public frc2::CommandHelper<frc2::Command, LoftShoot> {
public:
  /**
   * Sets shooter to values in networktable entries
   * 
   * @param shooter memory adress of shooter subsystem
  */
  explicit LoftShoot(ShooterSubsystem* shooter);

  void Initialize() override;
  void End(bool interrupted) override;

private:
// Declare local subsystems
ShooterSubsystem* m_shooter;

};