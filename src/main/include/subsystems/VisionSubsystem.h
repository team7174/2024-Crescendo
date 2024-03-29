// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>

#include <iostream>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem();

  void SetPose(std::string, frc::SwerveDrivePoseEstimator<4>*);
  void SetPoseLL3(frc::SwerveDrivePoseEstimator<4>*);
  void SetPoseLL2(frc::SwerveDrivePoseEstimator<4>*);
  void BlinkLEDs(bool blink);
  frc::Pose2d GetNoteLocation();
  bool SpeakerTags();

 private:
  std::shared_ptr<nt::NetworkTable> limelight2;
  std::shared_ptr<nt::NetworkTable> limelight3;
};