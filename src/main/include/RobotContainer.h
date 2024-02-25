// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc2/command/Commands.h>
// #include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include <subsystems/ArmSubsystem.h>
#include <subsystems/ClimbSubsystem.h>
#include <subsystems/ShooterSubsystem.h>
#include <subsystems/VisionSubsystem.h>
#include <subsystems/PDH.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer
{
public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  // The chooser for the autonomous routines
  // frc::SendableChooser<std::string> autonChooser;
  frc::SendableChooser<std::string> pathPlannerChooser;

  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_drive;
  ArmSubsystem m_armSubsystem;
  ClimbSubsystem m_climbSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  VisionSubsystem m_visionSubsystem;
  PDH m_PDH;

private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};

  // The secondary controller
  frc::XboxController m_secondaryController{OIConstants::kSecondaryControllerPort};

  void ConfigureButtonBindings();
};
