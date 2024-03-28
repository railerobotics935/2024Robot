
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "subsystems/ShooterSubsystem.h"
#include "subsystmes/StagerSubsystme.h"

/**
 * Command to take shooter speed and angle from nte and set the robot to shoot it
 */
class AmpShoot 
  : public frc2::CommandHelper<frc2::Command, AmpShoot> {
public:
  /**
   * Run the shooter so the note enters the amp
   * 
   * @param shooter memory adress of shooter subsystem
   * @param stager memory adress of stager subsystem
  */
  explicit AmpShoot(ShooterSubsystem* shooter, StagerSubsystem* stager);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:
// Declare local subsystems
ShooterSubsystem* m_shooter;
StagerSubsystem* m_stager;
};