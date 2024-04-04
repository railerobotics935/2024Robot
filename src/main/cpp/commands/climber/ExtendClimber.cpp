
#include "Constants.h"
#include "commands/climber/ExtendClimber.h"

ExtendClimber::ExtendClimber(ClimberSubsystem* climber) : m_climber{climber} {
  AddRequirements(m_climber);
}

void ExtendClimber::Execute() {
#ifdef PRINTDEBUG
  std::cout << "ExtendClimber Initialized\r\n";
#endif

  m_climber->SetClimberPower(-1.0);
}

void ExtendClimber::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "ExtendClimber Ended\r\n";
#endif

  m_climber->SetClimberPower(0.0);
}