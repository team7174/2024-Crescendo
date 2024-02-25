// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once
#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Translation2d.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <iostream>

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem();

  void SetPose(std::string, frc::SwerveDrivePoseEstimator<4>*);
  void SetPoseLL3(frc::SwerveDrivePoseEstimator<4>*);
  void SetPoseLL2(frc::SwerveDrivePoseEstimator<4>*);

  private:
    std::shared_ptr<nt::NetworkTable> limelight2;
    std::shared_ptr<nt::NetworkTable> limelight3;
};