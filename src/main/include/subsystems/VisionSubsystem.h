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

#include <iostream>

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem();
  frc::Translation2d GetPoseLL3();
  frc::Translation2d GetPoseLL2();
  frc::Translation2d ConvertToTranslation2d(std::vector<double> pose);

  private:
    std::shared_ptr<nt::NetworkTable> limelight2;
    std::shared_ptr<nt::NetworkTable> limelight3;
};