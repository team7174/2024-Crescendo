// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <units/angle.h>
#include <units/velocity.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() : m_drive(&m_visionSubsystem), m_armSubsystem(&m_drive), m_shooterSubsystem(&m_armSubsystem, &m_drive, &m_visionSubsystem, &m_secondaryController, &m_driverController) {
  // Initialize all of your commands and subsystems here

  // Register Named Commands.
  auto resetYaw = frc2::cmd::RunOnce([this] { m_drive.ZeroHeading(); });

  auto intakeDrop = frc2::cmd::RunOnce([this] { m_armSubsystem.SetDesiredAngle(ArmSubsystem::ArmStates::intake);
                                        m_shooterSubsystem.SetIntakeState(ShooterSubsystem::intakeStates::intake); });

  auto aimDrive = frc2::cmd::RunOnce([this] { m_drive.SetDriveState(DriveSubsystem::DriveStates::aimDrive); });

  auto regularDrive = frc2::cmd::RunOnce([this] { m_drive.SetDriveState(DriveSubsystem::DriveStates::joyStickDrive); });

  auto shootSpeaker = frc2::cmd::RunOnce([this] { m_armSubsystem.SetDesiredAngle(ArmSubsystem::ArmStates::autoAngle);
                                          m_shooterSubsystem.SetShooterState(ShooterSubsystem::shooterStates::shooterOn); });

  auto waitShoot = frc2::cmd::WaitUntil([this] { return (m_shooterSubsystem.currShooterState != ShooterSubsystem::shooterStates::shooterOn); }).WithTimeout(5_s);

  auto waitIntake = frc2::cmd::WaitUntil([this] { return (m_shooterSubsystem.currIntakeState != ShooterSubsystem::intakeStates::intake); }).WithTimeout(5_s);

  auto waitAngle = frc2::cmd::WaitUntil([this] { return (m_armSubsystem.ReachedDesiredAngle()); }).WithTimeout(2_s);

  // TODO: Test this
  pathplanner::NamedCommands::registerCommand("Reset Yaw", std::move(resetYaw));  // <- This example method returns CommandPtr
  pathplanner::NamedCommands::registerCommand("Intake Drop", std::move(intakeDrop));
  pathplanner::NamedCommands::registerCommand("Aim Drive", std::move(aimDrive));
  pathplanner::NamedCommands::registerCommand("Shoot Speaker", std::move(shootSpeaker));
  pathplanner::NamedCommands::registerCommand("Wait Shoot", std::move(waitShoot));
  pathplanner::NamedCommands::registerCommand("Wait Intake", std::move(waitIntake));
  pathplanner::NamedCommands::registerCommand("Wait Angle", std::move(waitAngle));
  pathplanner::NamedCommands::registerCommand("Regular Drive", std::move(regularDrive));

  // pathplanner::NamedCommands::registerCommand("NewTest", std::move(eventCmd));
  //  Set up default drive command
  //  The left stick controls translation of the robot.
  //  Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            units::meters_per_second_t{frc::ApplyDeadband(-m_driverController.GetLeftY(), 0.1) * DriveConstants::kMaxSpeed.value()},
            units::meters_per_second_t{frc::ApplyDeadband(-m_driverController.GetLeftX(), 0.1) * DriveConstants::kMaxSpeed.value()},
            units::radians_per_second_t{frc::ApplyDeadband(-m_driverController.GetRightX(), 0.1) * DriveConstants::kMaxAngularSpeed.value()},
            true);
      },
      {&m_drive}));

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::Trigger{[this]() { return m_driverController.GetYButtonPressed(); }}
      .OnTrue(frc2::cmd::RunOnce([this] { m_drive.ZeroHeading(); }));

  frc2::Trigger{[this]() { return m_driverController.GetRightTriggerAxis(); }}
      .OnTrue(frc2::cmd::RunOnce([this] { m_drive.SetDriveState(DriveSubsystem::DriveStates::aimDrive); }));

  frc2::Trigger{[this]() { return !m_driverController.GetRightTriggerAxis(); }}
      .OnTrue(frc2::cmd::RunOnce([this] { m_drive.SetDriveState(DriveSubsystem::DriveStates::joyStickDrive); }));

  frc2::Trigger{[this]() { return m_secondaryController.GetBButtonPressed(); }}
      .OnTrue(frc2::cmd::RunOnce([this] { m_armSubsystem.SetDesiredAngle(ArmSubsystem::ArmStates::intake); }));

  frc2::Trigger{[this]() { return m_secondaryController.GetLeftBumper(); }}
      .OnTrue(frc2::cmd::RunOnce([this] { m_shooterSubsystem.SetIntakeState(ShooterSubsystem::intakeStates::eject); }));

  frc2::Trigger{[this]() { return m_secondaryController.GetXButtonPressed(); }}
      .OnTrue(frc2::cmd::RunOnce([this] { m_armSubsystem.SetDesiredAngle(ArmSubsystem::ArmStates::upright);
                                          m_shooterSubsystem.SetShooterState(ShooterSubsystem::shooterStates::shooterEject); }));

  frc2::Trigger{[this]() { return m_secondaryController.GetLeftTriggerAxis(); }}
      .OnTrue(frc2::cmd::RunOnce([this] { m_shooterSubsystem.SetIntakeState(ShooterSubsystem::intakeStates::intake);
                                          m_armSubsystem.SetDesiredAngle(ArmSubsystem::ArmStates::intake); }));

  frc2::Trigger{[this]() { return m_secondaryController.GetRightTriggerAxis(); }}
      .OnTrue(frc2::cmd::RunOnce([this] { m_shooterSubsystem.SetShooterState(ShooterSubsystem::shooterStates::shooterOn);
                                          m_armSubsystem.SetDesiredAngle(ArmSubsystem::ArmStates::autoAngle); }));

  // mahdi figured right trigger would be better for amp so we decided to put this on D-pad? not sure where else it could go
  frc2::Trigger{[this]() { return m_secondaryController.GetRightBumper(); }}
      .OnTrue(frc2::cmd::RunOnce([this] { m_shooterSubsystem.SetShooterState(ShooterSubsystem::shooterStates::shooterStop);
                                          m_shooterSubsystem.SetIntakeState(ShooterSubsystem::intakeStates::stop); }));

  frc2::Trigger{[this]() { return m_secondaryController.GetYButton(); }}
      .OnTrue(frc2::cmd::RunOnce([this] { m_climbSubsystem.SetClimbState(ClimbSubsystem::ClimbStates::extend); }));

  frc2::Trigger{[this]() { return m_secondaryController.GetAButton(); }}
      .OnTrue(frc2::cmd::RunOnce([this] { m_climbSubsystem.SetClimbState(ClimbSubsystem::ClimbStates::retract); }));
  // amp code - were trying to figure out which controller to put it on
  // frc2::Trigger{[this]()
  //               { return m_secondaryController.GetRightBumper(); }}
  //     .OnTrue(frc2::cmd::RunOnce([this]
  //                                { m_shooterSubsystem.SetShooterState(ShooterSubsystem::shooterStates::shooterOn);
  //                                  m_armSubsystem.SetDesiredAngle(ArmSubsystem::ArmStates::upright); }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return pathplanner::PathPlannerAuto(pathPlannerChooser.GetSelected()).ToPtr();
}