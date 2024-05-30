
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "subsystems/GuftSubsystem.h"

/**
 * Command that shoots the note with the robot up to the amp
 */
class DefaultGuft 
  : public frc2::CommandHelper<frc2::Command, DefaultGuft> {
public:
  /**
   * Sets shooter to values in networktable entries
   * 
   * @param shooter memory adress of shooter subsystem
  */
  explicit DefaultGuft(GuftSubsystem* guft);

  void Initialize() override;
  void End(bool interrupted) override;

private:
// Declare local subsystems
GuftSubsystem* m_guft;

};