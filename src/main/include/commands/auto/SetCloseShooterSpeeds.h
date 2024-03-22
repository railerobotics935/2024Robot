
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include <subsystems/ShooterSubsystem.h>

class SetCloseShooterSpeeds
  : public frc2::CommandHelper<frc2::Command, SetCloseShooterSpeeds> {
public:
  /**
   * Creates a new SetCloseShooterSpeeds.
   * 
   * @param shooter The pointer to the shooter subsystem
   * @param opController The pointer to the operator controller
  */
  explicit SetCloseShooterSpeeds(ShooterSubsystem* shooter);

  void Initialize() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:
  ShooterSubsystem* m_shooter;
  bool m_ran = false;
};