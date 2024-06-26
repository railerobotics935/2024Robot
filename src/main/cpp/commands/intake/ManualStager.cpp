#include <frc/MathUtil.h>

#include "Constants.h"
#include "commands/intake/ManualStager.h"

ManualStager::ManualStager(StagerSubsystem* stager, frc::XboxController* operatorController) : m_stager{stager}, m_operatorController{operatorController} {
  AddRequirements(m_stager);
}

void ManualStager::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "ManualStager Initialized\r\n";
#endif
}

void ManualStager::Execute() {

#ifdef USEXBOXCONTROLLER
  if (frc::ApplyDeadband(m_operatorController->GetRawAxis(ControllerConstants::kStagerIntakeTrigger), 0.05) != 0.0)
    m_stager->SetMotorPower(-frc::ApplyDeadband(m_operatorController->GetRawAxis(ControllerConstants::kStagerIntakeTrigger), 0.05));
  else
    m_stager->SetMotorPower(frc::ApplyDeadband(m_operatorController->GetRawAxis(ControllerConstants::kStagerOuttakeTrigger), 0.05));
#else
  if (m_operatorController->GetRawButton(ControllerConstants::kStagerIntakeTrigger))
    m_stager->SetMotorPower(-1.0);
  else if (m_operatorController->GetRawButton(ControllerConstants::kStagerOuttakeTrigger))
    m_stager->SetMotorPower(1.0);
  else 
    m_stager->SetMotorPower(0.0);
#endif
}

void ManualStager::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "ManualStager Ended\r\n";
#endif
  m_stager->SetMotorPower(0.0);
}