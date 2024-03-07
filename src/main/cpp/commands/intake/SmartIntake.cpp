
#include "commands/intake/SmartIntake.h"


SmartIntake::SmartIntake(IntakeSubsystem* intake, StagerSubsystem* stager) {
  // Initilize local copys of pointers
  m_intake = intake;
  m_stager = stager;

  // Add reqierments for the command
  AddRequirements(m_intake);
  AddRequirements(m_stager);
}

void SmartIntake::Initialize() {
  m_intake->SetMotorPower(0.75);
  m_stager->SetMotorPower(0.5);
}

bool SmartIntake::IsFinished() {
  if (m_stager->NoteLoaded()) {
    return true;
  }
  else
    return false;
}

void SmartIntake::End(bool interrupted) {
  m_intake->SetMotorPower(0.0);
  m_stager->SetMotorPower(0.0);
}