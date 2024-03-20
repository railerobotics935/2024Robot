
#include "Constants.h"
#include "commands/climber/StopClimber.h"

StopClimber::StopClimber(ClimberSubsystem* climber) : m_climber{climber} {
  AddRequirements(m_climber);
}

void StopClimber::Initialize() {
#ifdef PRINTDEBUG 
  std::cout << "StopClimber Initialized\r\n";
#endif

  m_climber->SetClimberPower(0.0);
}

void StopClimber::End(bool interrupted) {
#ifdef PRINTDEBUG 
  std::cout << "StopClimber Ended\r\n";
#endif

  m_climber->SetClimberPower(0.0);
}