#include "subsystems/VisionSubsystem.h"

VisionSubsystem::VisionSubsystem()
{
  this->limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

frc::Translation2d VisionSubsystem::GetPosition()
{
  auto limelight3BotPose = this->limelight->GetNumberArray("botpose_wpiblue", {});
  if (limelight3BotPose.size() >= 2)
  {
    double x = limelight3BotPose[0];
    double y = limelight3BotPose[1];
    frc::SmartDashboard::PutNumber("Limelight: botpose x", x);
    frc::SmartDashboard::PutNumber("Limelight: botpose y", y);

    units::meter_t xMeters(x);
    units::meter_t yMeters(y);

    frc::Translation2d position(xMeters, yMeters);
    // TODO: FIXME always return something even if size < 2??? Maybe pair with a boolean as the first obj?
    return position;
  }
}
