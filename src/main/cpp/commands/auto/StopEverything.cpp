
#include "commands/auto/StopEverything.h"
#include "Constants.h"

StopEverything::StopEverything(StagerSubsystem* stager, ShooterSubsystem* shooter) {
  // Initialize local copies of pointers
  m_stager = stager;
  m_shooter = shooter;
  // Add requirements for the command
  AddRequirements(m_stager);
  AddRequirements(m_shooter);
}

void StopEverything::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "StopEverything Initialized\r\n";
#endif
  m_stager->SetMotorPower(0.0);
  m_shooter->SetShooterMotorPower(0.0);
  m_ran = true;
}

bool StopEverything::IsFinished() {
  if (m_ran == true)
    return true;
  else
    return false;
}

void StopEverything::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "StopEverything Ended\r\n";
#endif
  m_ran = false;
}