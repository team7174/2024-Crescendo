#include "subsystems/VisionSubsystem.h"

VisionSubsystem::VisionSubsystem() {
}

void VisionSubsystem::SetPose(std::string table_name, frc::SwerveDrivePoseEstimator<4> *m_odometry) {
  double xyStds;
  units::angle::radian_t degStds;

  std::shared_ptr<nt::NetworkTable> ll = nt::NetworkTableInstance::GetDefault().GetTable(table_name);
  auto llBotPoseEntry = ll->GetEntry("botpose_wpiblue");
  auto llBotPose = llBotPoseEntry.GetDoubleArray({});

  // weirdness
  if (llBotPose.size() < 6) {
    return;
  }
  frc::Pose2d visionBotPose = frc::Pose2d(
      frc::Translation2d(units::length::meter_t(llBotPose[0]), units::length::meter_t(llBotPose[1])),
      frc::Rotation2d(units::angle::radian_t(llBotPose[5] * (M_PI / 180.0))));

  // getlastchange() in microseconds, ll latency in milliseconds
  auto visionTime = units::time::second_t((llBotPoseEntry.GetLastChange() / 1000000.0) - (llBotPose[6] / 1000.0));
  // auto visionTime = frc::Timer::GetFPGATimestamp() - (llBotPose[6]/1000.0)

  // distance from current pose to vision estimated pose
  units::meter_t poseDifference = m_odometry->GetEstimatedPosition().Translation().Distance(visionBotPose.Translation());

  int tagCount = (int)llBotPose[7];
  double tagArea = llBotPose[10];
  // multiple targets detected
  if (tagCount >= 2) {
    xyStds = 0.5;
    degStds = units::angle::radian_t(6_deg);
  }
  // 1 target with large area and close to estimated pose
  else if (tagArea > 0.8 && poseDifference < 0.5_m) {
    xyStds = 1.0;
    degStds = units::angle::radian_t(12_deg);
  }
  // 1 target farther away and estimated pose is close
  else if (tagArea > 0.1 && poseDifference < 0.3_m) {
    xyStds = 2.0;
    degStds = units::angle::radian_t(30_deg);
  }
  // conditions don't match to add a vision measurement
  else {
    return;
  }

  m_odometry->SetVisionMeasurementStdDevs({xyStds, xyStds, degStds.value()});
  m_odometry->AddVisionMeasurement(visionBotPose, visionTime);
}

void VisionSubsystem::SetPoseLL3(frc::SwerveDrivePoseEstimator<4> *m_odometry) {
  SetPose("limelight-llthree", m_odometry);
}

void VisionSubsystem::SetPoseLL2(frc::SwerveDrivePoseEstimator<4> *m_odometry) {
  SetPose("limelight-lltwo", m_odometry);
}

frc::Pose2d VisionSubsystem::GetNoteLocation() {
  std::shared_ptr<nt::NetworkTable> ll = nt::NetworkTableInstance::GetDefault().GetTable("limelight-lltwo");
  auto noteX = units::meter_t(ll->GetNumber("tx", 0.0));
  auto noteY = units::meter_t(ll->GetNumber("ty", 0.0));
  return frc::Pose2d(noteX, noteY, 0_deg);
}

bool VisionSubsystem::SpeakerTags() {
  std::shared_ptr<nt::NetworkTable> ll = nt::NetworkTableInstance::GetDefault().GetTable("limelight-llthree");
  // auto llBotPoseEntry = ll->GetEntry("botpose");
  auto targetID = ll->GetEntry("tid").GetDouble(0.0);
  if ((targetID == 3 || targetID == 4 || targetID == 7 || targetID == 8)) {  //&& llBotPoseEntry.GetDoubleArray({})[7] >= 2.0
    frc::SmartDashboard::PutBoolean("Speaker Tags", true);
    return true;
  }
  frc::SmartDashboard::PutBoolean("Speaker Tags", false);
  return false;
}

void VisionSubsystem::BlinkLEDs(bool blink) {
  std::shared_ptr<nt::NetworkTable> ll2 = nt::NetworkTableInstance::GetDefault().GetTable("limelight-lltwo");
  std::shared_ptr<nt::NetworkTable> ll3 = nt::NetworkTableInstance::GetDefault().GetTable("limelight-llthree");

  if (blink) {
    ll2->PutNumber("ledMode", 2);
    ll3->PutNumber("ledMode", 2);
  } else {
    ll2->PutNumber("ledMode", 1);
    ll3->PutNumber("ledMode", 1);
  }
}