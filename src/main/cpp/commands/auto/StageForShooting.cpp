
#include "commands/auto/StageForShooting.h"
#include "Constants.h"

StageForShooting::StageForShooting(StagerSubsystem* stager) {
  // Initialize local copies of pointers
  m_stager = stager;
  // Add requirements for the command
  AddRequirements(m_stager);
}

void StageForShooting::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "StageForShooting Initialized\r\n";
#endif
  m_stager->SetMotorPower(1.0);
  m_ran = true;
}

bool StageForShooting::IsFinished() {
  if (m_ran == true)
    return true;
  else
    return false;
}

void StageForShooting::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "StageForShooting Ended\r\n";
#endif
  m_ran = false;
}