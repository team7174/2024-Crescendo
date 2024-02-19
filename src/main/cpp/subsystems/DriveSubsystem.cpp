// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include <frc/geometry/Rotation2d.h>

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem(VisionSubsystem *passedVisionSubsystem)
    : m_frontLeft{kFrontLeftDriveMotorPort,
                  kFrontLeftTurningMotorPort,
                  kFrontLeftTurningEncoderPorts,
                  kFrontLeftEncoderOffset},

      m_rearLeft{kRearLeftDriveMotorPort,
                 kRearLeftTurningMotorPort,
                 kRearLeftTurningEncoderPorts,
                 kRearLeftEncoderOffset},

      m_frontRight{kFrontRightDriveMotorPort,
                   kFrontRightTurningMotorPort,
                   kFrontRightTurningEncoderPorts,
                   kFrontRightEncoderOffset},

      m_rearRight{kRearRightDriveMotorPort,
                  kRearRightTurningMotorPort,
                  kRearRightTurningEncoderPorts,
                  kRearRightEncoderOffset},

      m_odometry{kDriveKinematics,
                 m_gyro.GetRotation2d(),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}}
{
  m_visionSubsystem = passedVisionSubsystem;
  m_aimController.SetTolerance(1);
  m_aimController.EnableContinuousInput(-180, 180);

  // Configure the AutoBuilder last
  pathplanner::AutoBuilder::configureHolonomic(
      [this]()
      { return this->GetPose(); }, // Robot pose supplier
      [this](frc::Pose2d pose)
      { this->ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
      [this]()
      { return this->kDriveKinematics.ToChassisSpeeds(this->GetModuleStates()); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      [this](frc::ChassisSpeeds speeds)
      { this->Drive(speeds.vx, speeds.vy, speeds.omega, false); },           // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      pathplanner::HolonomicPathFollowerConfig(                              // HolonomicPathFollowerConfig, this should likely live in your Constants class
          pathplanner::PIDConstants(AutoConstants::kPXController, 0.0, 0.0), // Translation PID constants
          pathplanner::PIDConstants(AutoConstants::kPYController, 0.0, 0.0), // Rotation PID constants
          AutoConstants::kMaxSpeed,                                          // Max module speed, in m/s
          this->kModuleRadius,                                               // Drive base radius in meters. Distance from robot center to furthest module.
          pathplanner::ReplanningConfig()                                    // Default path replanning config. See the API for the options here
          ),
      []()
      {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        auto ally = frc::DriverStation::GetAlliance();
        if (ally)
        {
          return ally.value() == frc::DriverStation::Alliance::kRed;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
  );

  frc::SmartDashboard::PutData("Field", &m_field);
}

void DriveSubsystem::Periodic()
{
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});

  UpdatePoseLimelight(m_visionSubsystem->GetPoseLL2());
  UpdatePoseLimelight(m_visionSubsystem->GetPoseLL3());

  m_field.SetRobotPose(m_odometry.GetEstimatedPosition());
  getShootingValues();

  // /*
  //  * This will get the simulated sensor readings that we set
  //  * in the previous article while in simulation, but will use
  //  * real values on the robot itself.
  //  */
  // m_odometry.Update(m_gyro.GetRotation2d(),
  //                   rotationsToMeters(m_leftLeader.GetPosition().GetValue()),
  //                   rotationsToMeters(m_rightLeader.GetPosition().GetValue()));
  // m_field.SetRobotPose(m_odometry.GetPose());
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative)
{
  frc::SmartDashboard::PutNumber("Drive X", xSpeed.value());
  frc::SmartDashboard::PutNumber("Drive Y", ySpeed.value());

  if (m_desiredDriveState == DriveStates::aimDrive)
  {
    rot = units::radians_per_second_t(std::clamp(m_aimController.Calculate(m_gyro.GetRotation2d().Degrees().value()), -AutoConstants::kMaxAngularSpeed.value(), AutoConstants::kMaxAngularSpeed.value()));
  }

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetDriveState(DriveStates desiredDriveState)
{
  m_desiredDriveState = desiredDriveState;
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates)
{
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

wpi::array<frc::SwerveModuleState, 4> DriveSubsystem::GetModuleStates()
{
  return wpi::array{
      m_frontLeft.GetState(),
      m_frontRight.GetState(),
      m_rearLeft.GetState(),
      m_rearRight.GetState()};
}

void DriveSubsystem::ResetEncoders()
{
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading()
{
  return m_gyro.GetRotation2d().Degrees();
}

void DriveSubsystem::ZeroHeading()
{
  m_gyro.Reset();
}

double DriveSubsystem::GetTurnRate()
{
  return -m_gyro.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose()
{
  return m_odometry.GetEstimatedPosition();
}

frc::Translation3d DriveSubsystem::GetSpeakerCenter()
{
  frc::Translation3d speakerCenter;
  if (auto ally = frc::DriverStation::GetAlliance())
  {
    if (ally.value() == frc::DriverStation::Alliance::kRed)
    {
      allianceColorBlue = false;
    }
    else
    {
      allianceColorBlue = true;
    }
  }
  units::meter_t X = 0.5 * (topLeftSpeaker.X() + bottomRightSpeaker.X());
  units::meter_t Y = 0.5 * (topLeftSpeaker.Y() + bottomRightSpeaker.Y());
  units::meter_t Z = 0.5 * (topLeftSpeaker.Z() + bottomRightSpeaker.Z());
  if (!allianceColorBlue)
  {
    X = 16.5410642_m - (0.5 * (topLeftSpeaker.X() + bottomRightSpeaker.X()));
  }
  frc::SmartDashboard::PutNumber("Speaker X", X.value());
  frc::SmartDashboard::PutNumber("Speaker Y", Y.value());
  frc::SmartDashboard::PutNumber("Speaker Z", Z.value());
  return {X, Y, Z};
}

std::pair<double, double> DriveSubsystem::getShootingValues()
{
  auto speakerCenter = GetSpeakerCenter();
  frc::Transform2d robotToSpeaker = frc::Transform2d{GetPose(), frc::Pose2d(speakerCenter.ToTranslation2d(), frc::Rotation2d())};
  frc::Translation2d robotToSpeakerTranslation = robotToSpeaker.Translation();
  double shootingDistance = robotToSpeakerTranslation.Norm().value();
  double aimAngle = robotToSpeakerTranslation.Angle().Degrees().value();

  // auto m_robotPose = GetPose();
  // auto xOffset = m_robotPose.X().value() - speakerX;
  // auto yOffset = m_robotPose.Y().value() - 5.547868;
  // auto angle = atan(abs(yOffset) / abs(xOffset));
  // angle = angle * (180 / M_PI);

  // if (allianceColor == frc::DriverStation::Alliance::kRed)
  // {
  //   if (yOffset > 0) // only on red side
  //   {
  //     angle = -angle;
  //   }
  // }
  // else
  // {
  //   if (yOffset < 0) // only on blue side
  //   {
  //     angle = -angle;
  //   }
  // }

  // if (yOffset < 0) // only on blue side
  // {
  //   angle = -angle;
  // }
  frc::SmartDashboard::PutNumber("Aim Angle", aimAngle);
  frc::SmartDashboard::PutNumber("Shooting Distance", shootingDistance);
  m_aimController.SetSetpoint(0);
  return std::pair<double, double>(shootingDistance, aimAngle);
}

void DriveSubsystem::UpdatePoseLimelight(frc::Translation2d pose)
{
  if (pose != ignorePose)
  {
    frc::Pose2d robotPose(pose, m_gyro.GetRotation2d());
    m_odometry.AddVisionMeasurement(robotPose, frc::Timer::GetFPGATimestamp());
  }
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}
