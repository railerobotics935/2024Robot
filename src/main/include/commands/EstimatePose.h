#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "subsystems/DriveSubsystem.h"

class EstimatePose 
  : public frc2::CommandHelper<frc2::Command, EstimatePose> {
public:
  /**
   * Command that handles when and how to include vision data into pose esitmation
   * This does not require any subsystems becasue it is not setting anything on any subsystem.
   * 
   * @param drive Pointer to the drive subsystem
  */
  EstimatePose(DriveSubsystem* drive);

  void Execute() override;

private:
  // Local copy of subsystems
  DriveSubsystem* m_drive;

};