// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/shuffleboard/Shuffleboard.h>

#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/JoystickButton.h>

#include <units/angle.h>
#include <units/velocity.h>

#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() : m_drive(), m_vision()
{
  // Initialize all of your commands and subsystems here

  // Register Named Commands.
  auto testCmd = frc2::cmd::Print("TESTING A COMMAND");
  auto eventCmd = frc2::cmd::Print("EVENT MARKER");

  // TODO: Test this
  pathplanner::NamedCommands::registerCommand("TEST CMD", std::move(testCmd)); // <- This example method returns CommandPtr
  pathplanner::NamedCommands::registerCommand("NewTest", std::move(eventCmd));
  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this]
      {
        m_drive.Drive(
            units::meters_per_second_t{frc::ApplyDeadband(-m_driverController.GetLeftY(), 0.08) * AutoConstants::kMaxSpeed.value()},
            units::meters_per_second_t{frc::ApplyDeadband(-m_driverController.GetLeftX(), 0.08) * AutoConstants::kMaxSpeed.value()},
            units::radians_per_second_t{frc::ApplyDeadband(-m_driverController.GetRightX(), 0.08) * AutoConstants::kMaxAngularSpeed.value()},
            true);
      },
      {&m_drive}));

  m_armSubsystem = std::make_shared<ArmSubsystem>(&m_secondaryController);
  m_armSubsystem->SetDefaultCommand(frc2::RunCommand([this]
                                                     { m_armSubsystem->SetDesiredAngle(&m_secondaryController); },
                                                     {m_armSubsystem.get()}));

  m_climbSubsystem = std::make_shared<ClimbSubsystem>(&m_secondaryController);
  m_climbSubsystem->SetDefaultCommand(frc2::RunCommand([this]
                                                       { m_climbSubsystem->SetDesiredPosition(&m_secondaryController); },
                                                       {m_climbSubsystem.get()}));

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings()
{
  frc2::Trigger{[this]()
                { return m_driverController.GetAButtonPressed(); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { 
        auto position = m_vision.GetPosition(); 
        m_drive.UpdatePoseLimelight(position); }));

  frc2::Trigger{[this]()
                { return m_driverController.GetYButtonPressed(); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_drive.ZeroHeading(); }));

  // frc2::JoystickButton(
  //   &m_driverController,
  //   frc::XboxController::Button::kA).OnTrue(
  //     frc2::cmd::RunOnce([this]{ GetLimelightPose(); }));

  // frc2::JoystickButton(
  //   &m_driverController,
  //   frc::XboxController::Button::kY).OnTrue(
  //     frc2::cmd::RunOnce([this]{ m_drive.ZeroHeading(); }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  return pathplanner::PathPlannerAuto(pathPlannerChooser.GetSelected()).ToPtr();
}
