// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <frc/MathUtil.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>
#include <frc2/command/Commands.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this]
      {
        m_drive.Drive(
            units::meters_per_second_t{frc::ApplyDeadband(-m_driverController.GetLeftY(), 0.08) * AutoConstants::kMaxSpeed},
            units::meters_per_second_t{frc::ApplyDeadband(-m_driverController.GetLeftX(), 0.08) * AutoConstants::kMaxSpeed},
            units::radians_per_second_t{frc::ApplyDeadband(-m_driverController.GetRightX(), 0.08)} * AutoConstants::kMaxAngularSpeed.value(), true);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings()
{
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kA)
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { GetLimelightPose(); }));

  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kY)
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_drive.ZeroHeading(); }));
}

void RobotContainer::GetLimelightPose()
{
  std::shared_ptr<nt::NetworkTable> limelight3 = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  auto limelight3BotPose = limelight3->GetNumberArray("botpose_wpiblue", {});
  if (limelight3BotPose.size() >= 2)
  {
    double x = limelight3BotPose[0];
    double y = limelight3BotPose[1];
    frc::SmartDashboard::PutNumber("botpose x", x);
    frc::SmartDashboard::PutNumber("botpose y", y);

    units::meter_t xMeters(x);
    units::meter_t yMeters(y);

    frc::Translation2d position(xMeters, yMeters);
    m_drive.UpdatePoseLimelight(position);
  }
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  // Set up config for trajectory
  auto selectedPath = m_chooser.GetSelected();

  auto selectedTrajectory = frc::TrajectoryUtil::FromPathweaverJson(selectedPath);

  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);

  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  //   auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //       // Start at the origin facing the +X direction
  //       frc::Pose2d{0_m, 0_m, 0_deg},
  //       // Pass through these two interior waypoints, making an 's' curve path
  //       {frc::Translation2d{1_m, 0_m}, frc::Translation2d{1_m, 1_m}, frc::Translation2d{0_m, 1_m}},
  //       // End 3 meters straight ahead of where we started, facing forward
  //       frc::Pose2d{0_m, 0_m, 0_deg},
  //       // Pass the config
  //       config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      selectedTrajectory, [this]()
      { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController{AutoConstants::kPXController, 0, 0},
      frc2::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates)
      { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(selectedTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]()
          { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, true); },
          {}));
}
