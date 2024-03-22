
#include "commands/auto/StopStager.h"
#include "Constants.h"

StopStager::StopStager(StagerSubsystem* stager) {
  // Initialize local copies of pointers
  m_stager = stager;
  // Add requirements for the command
  AddRequirements(m_stager);
}

void StopStager::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "StopStager Initialized\r\n";
#endif
  m_stager->SetMotorPower(0.0);
  m_ran = true;
}

bool StopStager::IsFinished() {
  if (m_ran == true)
    return true;
  else
    return false;
}

void StopStager::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "StopStager Ended\r\n";
#endif
  m_ran = false;
}