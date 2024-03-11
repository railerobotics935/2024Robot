
#include "commands/intake/SmartOuttake.h"


SmartOuttake::SmartOuttake(IntakeSubsystem* intake, StagerSubsystem* stager) {
  // Initilize local copys of pointers
  m_intake = intake;
  m_stager = stager;

  // Add reqierments for the command
  AddRequirements(m_intake);
  AddRequirements(m_stager);
}

void SmartOuttake::Initialize() {
  m_intake->SetMotorPower(-0.75);
  m_stager->SetMotorPower(-0.5);
}

bool SmartOuttake::IsFinished() {
  if (m_stager->NoteLoaded()) {
    return true;
  }
  else
    return false;
}

void SmartOuttake::End(bool interrupted) {
  m_intake->SetMotorPower(0.0);
  m_stager->SetMotorPower(0.0);
}