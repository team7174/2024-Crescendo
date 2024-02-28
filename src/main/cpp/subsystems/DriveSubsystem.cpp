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

      profiledAimController(
          1.2, // Placeholder for proportional gain
          0.0, // Placeholder for integral gain
          0.0, // Placeholder for derivative gain
          frc::TrapezoidProfile<units::radian>::Constraints(AutoConstants::kMaxAngularSpeed, AutoConstants::kMaxAngularAcceleration)),

      m_odometry{kDriveKinematics,
                 GetHeading(),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}}
{
  m_visionSubsystem = passedVisionSubsystem;

  profiledAimController.EnableContinuousInput(-180_deg, 180.0_deg);
  profiledAimController.SetTolerance(5.0_deg);
  profiledAimController.SetGoal(180_deg);

  // Configure the AutoBuilder last
  pathplanner::AutoBuilder::configureHolonomic(
      [this]()
      { return this->GetPose(); }, // Robot pose supplier
      [this](frc::Pose2d pose)
      { this->ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
      [this]()
      { return this->kDriveKinematics.ToChassisSpeeds(this->GetModuleStates()); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      [this](frc::ChassisSpeeds speeds)
      { this->Drive(speeds.vx, speeds.vy, speeds.omega, true); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      pathplanner::HolonomicPathFollowerConfig(                   // HolonomicPathFollowerConfig, this should likely live in your Constants class
          pathplanner::PIDConstants(AutoConstants::kPXController, 0.0, 0.0),
          pathplanner::PIDConstants(AutoConstants::kPThetaController, 0.0, 0.0), // Rotation PID constants
          AutoConstants::kMaxSpeed,                                              // Max module speed, in m/s
          this->kModuleRadius,                                                   // Drive base radius in meters. Distance from robot center to furthest module.
          pathplanner::ReplanningConfig()                                        // Default path replanning config. See the API for the options here
          ),
      []()
      {
        auto ally = frc::DriverStation::GetAlliance();
        return ally.value() == frc::DriverStation::Alliance::kRed;
      },
      this // Reference to this subsystem to set requirements
  );

  frc::SmartDashboard::PutData("Field", &m_field);
}

void DriveSubsystem::Periodic()
{
  atShootingAngle();
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(GetHeading(),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});

  m_visionSubsystem->SetPoseLL3(&m_odometry);
  // m_visionSubsystem->SetPoseLL2(&m_odometry);
  if (m_desiredDriveState == aimDrive)
  {
    Drive(units::meters_per_second_t(0), units::meters_per_second_t(0), units::radians_per_second_t(0), true);
  }
  m_field.SetRobotPose(GetPose());
  getShootingValues();
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative)
{
  frc::SmartDashboard::PutNumber("Drive X", xSpeed.value());
  frc::SmartDashboard::PutNumber("Drive Y", ySpeed.value());
  frc::SmartDashboard::PutNumber("Drive Z", rot.value());

  if (!allianceColorBlue)
  {
    xSpeed = -xSpeed;
    ySpeed = -ySpeed;
  }

  if (m_desiredDriveState == DriveStates::aimDrive)
  {
    rot = units::radians_per_second_t(frc::ApplyDeadband(-std::clamp(profiledAimController.Calculate(units::degree_t(getShootingValues().second)), -AutoConstants::kMaxAngularSpeed.value(), AutoConstants::kMaxAngularSpeed.value()), 0.03));
  }

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, GetPose().Rotation())
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
  if (allianceColorBlue)
  {
    return m_gyro.GetRotation2d().Degrees();
  }
  else
  {
    return (m_gyro.GetRotation2d().Degrees() + 180_deg);
  }
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

bool DriveSubsystem::atShootingAngle()
{
  double angleOffset = abs(180 - abs(getShootingValues().second));
  frc::SmartDashboard::PutBoolean("Speaker Aim", angleOffset < 2.5);
  if (angleOffset < 2.5)
  {
    return true;
  }
  return false;
}

frc::Translation3d DriveSubsystem::GetSpeakerCenter()
{
  frc::Translation3d speakerCenter;
  if (auto ally = frc::DriverStation::GetAlliance())
  {
    if (ally.value() == frc::DriverStation::Alliance::kRed)
    {
      allianceColorBlue = false;
      profiledAimController.SetGoal(180_deg);
    }
    else
    {
      allianceColorBlue = true;
      profiledAimController.SetGoal(180_deg);
    }
  }
  units::meter_t X = 0.5 * (topLeftSpeaker.X() + bottomRightSpeaker.X());
  units::meter_t Y = 0.5 * (topLeftSpeaker.Y() + bottomRightSpeaker.Y());
  units::meter_t Z = 0.5 * (topLeftSpeaker.Z() + bottomRightSpeaker.Z());
  if (!allianceColorBlue)
  {
    X = 16.5410642_m - (0.5 * (topLeftSpeaker.X() + bottomRightSpeaker.X()));
  }
  return {X, Y, Z};
}

std::pair<double, double> DriveSubsystem::getShootingValues()
{
  frc::Transform2d robotToSpeaker = frc::Transform2d{GetPose(), frc::Pose2d(GetSpeakerCenter().ToTranslation2d(), frc::Rotation2d())};
  frc::Translation2d robotToSpeakerTranslation = robotToSpeaker.Translation();
  double shootingDistance = robotToSpeakerTranslation.Norm().value();
  double aimAngle = robotToSpeakerTranslation.Angle().Degrees().value();
  frc::SmartDashboard::PutNumber("Aim Angle", aimAngle);
  frc::SmartDashboard::PutNumber("Shooting Distance", shootingDistance);
  return std::pair<double, double>(shootingDistance, aimAngle);
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}
