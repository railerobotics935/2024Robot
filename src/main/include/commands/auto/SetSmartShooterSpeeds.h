
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

class SetSmartShooterSpeeds
  : public frc2::CommandHelper<frc2::Command, SetSmartShooterSpeeds> {
public:
  /**
   * Creates a new SetSmartShooterSpeeds.
   * 
   * @param shooter The pointer to the shooter subsystem
   * @param opController The pointer to the operator controller
  */
  explicit SetSmartShooterSpeeds(ShooterSubsystem* shooter, DriveSubsystem* drive);

  void Initialize() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:
  ShooterSubsystem* m_shooter;
  DriveSubsystem* m_drive;
  
  double m_robotDistance;
  bool m_ran = false;

};