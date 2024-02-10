#include "subsystems/VisionSubsystem.h"

VisionSubsystem::VisionSubsystem()
{}

frc::Translation2d VisionSubsystem::GetPoseLL3()
{
  limelight3 = nt::NetworkTableInstance::GetDefault().GetTable("LL3");
  auto limelight3BotPose = limelight3->GetNumberArray("botpose_wpiblue", {});
  return ConvertToTranslation2d(limelight3BotPose);
}

frc::Translation2d VisionSubsystem::GetPoseLL2()
{
  limelight2 = nt::NetworkTableInstance::GetDefault().GetTable("LL2");
  auto limelight2BotPose = limelight2->GetNumberArray("botpose_wpiblue", {});
  return ConvertToTranslation2d(limelight2BotPose);
}

frc::Translation2d VisionSubsystem::ConvertToTranslation2d(std::vector<double> pose)
{
  if (pose.size() >= 2)
  {
    units::meter_t xMeters(pose[0]);
    units::meter_t yMeters(pose[1]);
    frc::Translation2d position(xMeters, yMeters);
    return position;
  }
  else
  {
    frc::Translation2d position(0_m, 0_m);
    return position;
  }
}
