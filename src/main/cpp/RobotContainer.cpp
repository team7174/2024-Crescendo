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

RobotContainer::RobotContainer() : m_drive(&m_visionSubsystem), m_shooterSubsystem(&m_armSubsystem)
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

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings()
{
  frc2::Trigger{[this]()
                { return m_driverController.GetYButtonPressed(); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_drive.ZeroHeading(); }));

  frc2::Trigger{[this]()
                { return m_secondaryController.GetXButtonPressed(); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_armSubsystem.SetDesiredAngle(ArmSubsystem::ArmStates::intake); }));

  frc2::Trigger{[this]()
                { return m_secondaryController.GetLeftBumper(); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_shooterSubsystem.SetIntakeState(ShooterSubsystem::intakeStates::eject);
                                   m_shooterSubsystem.SetShooterState(ShooterSubsystem::shooterStates::shooterEject); }));

  frc2::Trigger{[this]()
                { return m_secondaryController.GetBButtonPressed(); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_armSubsystem.SetDesiredAngle(ArmSubsystem::ArmStates::upright); }));

  frc2::Trigger{[this]()
                { return m_secondaryController.GetLeftTriggerAxis(); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_shooterSubsystem.SetIntakeState(ShooterSubsystem::intakeStates::intake); }));

  frc2::Trigger{[this]()
                { return m_secondaryController.GetRightTriggerAxis(); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_shooterSubsystem.SetShooterState(ShooterSubsystem::shooterStates::shooterOn); }));

  frc2::Trigger{[this]()
                { return m_secondaryController.GetRightBumper(); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_shooterSubsystem.SetShooterState(ShooterSubsystem::shooterStates::shooterStop); }));

  frc2::Trigger{[this]()
                { return m_secondaryController.GetYButton(); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_climbSubsystem.SetClimbState(ClimbSubsystem::ClimbStates::extend); }));

  frc2::Trigger{[this]()

                { return m_secondaryController.GetAButton(); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_climbSubsystem.SetClimbState(ClimbSubsystem::ClimbStates::retract); }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  return pathplanner::PathPlannerAuto(pathPlannerChooser.GetSelected()).ToPtr();
}
