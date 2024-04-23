
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "subsystems/ShooterSubsystem.h"
#include "subsystems/DriveSubsystem.h"

/**
 * Command to take shooter speed and angle from nte and set the robot to shoot it
 */
class DefaultShooter 
  : public frc2::CommandHelper<frc2::Command, DefaultShooter> {
public:
  /**
   * Creates a new Deafault Shooter
   * 
   * @param shooter memory adress of shooter subsystem
  */
  explicit DefaultShooter(ShooterSubsystem* shooter, DriveSubsystem* drive, frc::XboxController* opController);

  void Initialize() override;
  void End(bool interrupted) override;

private:
// Declare local subsystems
ShooterSubsystem* m_shooter;
DriveSubsystem* m_drive;
frc::XboxController* m_opController;
};