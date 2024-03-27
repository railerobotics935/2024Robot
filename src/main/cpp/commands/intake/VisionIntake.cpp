
#include "commands/intake/VisionIntake.h"


VisionIntake::VisionIntake(IntakeSubsystem* intake, StagerSubsystem* stager, DriveSubsystem* drive) {
  // Initilize local copys of pointers
  m_intake = intake;
  m_stager = stager;
  m_drive = drive;

  // Add reqierments for the command
  AddRequirements(m_intake);
  AddRequirements(m_stager);
}

void VisionIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "VisionIntake Initialize\r\n";
#endif
  m_pathPoses.clear();
  // run command to get best note id and grab the pose at the same time
  m_pathPoses = {m_drive->GetPose(), frc::Pose2d{m_drive->GetFieldRelativeTranslation(m_drive->GetBestNoteId()), m_drive->GetPose().Rotation()}};


}

bool VisionIntake::IsFinished() {
  if (m_stager->NoteLoaded()) {
    return true;
  }
  else
    return false;
}
void VisionIntake::End(bool interrupted) {
  m_intake->SetMotorPower(0.0);
  m_stager->SetMotorPower(0.0);
  m_drive->Drive((units::meters_per_second_t)0.0, (units::meters_per_second_t)0.0, (units::radians_per_second_t)0.0, true);
#ifdef PRINTDEBUG
  std::cout << "VisionIntake End\r\n";
#endif
}
