// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

  frc::Translation2d GetPosition();

  private:
    std::shared_ptr<nt::NetworkTable> limelight;
};