// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#pragma once

namespace DriveConstants
{
    constexpr int kFrontLeftDriveMotorPort = 10;
    constexpr int kRearLeftDriveMotorPort = 20;
    constexpr int kFrontRightDriveMotorPort = 40;
    constexpr int kRearRightDriveMotorPort = 30;

    constexpr int kFrontLeftTurningMotorPort = 11;
    constexpr int kRearLeftTurningMotorPort = 21;
    constexpr int kFrontRightTurningMotorPort = 41;
    constexpr int kRearRightTurningMotorPort = 31;

    constexpr int kFrontLeftTurningEncoderPorts = 1;
    constexpr int kRearLeftTurningEncoderPorts = 2;
    constexpr int kFrontRightTurningEncoderPorts = 0;
    constexpr int kRearRightTurningEncoderPorts = 3;

    constexpr double kFrontLeftEncoderOffset = 0.01107894677394;
    constexpr double kRearLeftEncoderOffset = 0.34623807543156;
    constexpr double kFrontRightEncoderOffset = 0.0924851208955030;
    constexpr double kRearRightEncoderOffset = 0.922683588934038;

} // namespace DriveConstants

namespace ModuleConstants
{
    constexpr double kWheelDiameterMeters = 0.1016;
    constexpr double kWheelCircumference = kWheelDiameterMeters * std::numbers::pi;
    constexpr double driveGearRatio = 6.12;
    constexpr double angleGearRatio = 12.8;

    constexpr double kPModuleTurningController = 1;
    constexpr double kPModuleDriveController = 1;
} // namespace ModuleConstants

namespace StormbreakerConstants
{
    constexpr int armEncoderID = 8;
    constexpr int leftArmID = 50;
    constexpr int rightArmID = 51;
    constexpr double armEncoderOffset = 0.5901480147537;
    constexpr double armkP = 0.03;
    constexpr double armkI = 0.0;
    constexpr double armkD = 0.0;
    constexpr double armGearRatio = 433.3333333333;
    constexpr double armLength = 0.70377;
    constexpr auto kArmAngleSpeed = 18_deg_per_s;
    constexpr auto kArmAngleAcceleration = 15_deg_per_s_sq;
} // namespace StormbreakerConstants

namespace ClimberConstants
{
    constexpr int leftClimbID = 60;
    constexpr int rightClimbID = 61;
    constexpr double climbkP = 0.025;
    constexpr double climbkI = 0.0;
    constexpr double climbkD = 0.0;
} // namespace ClimberConstants

namespace ShooterConstants
{
    constexpr int leftShooterID = 55;
    constexpr int rightShooterID = 56;
    constexpr int intakeID = 57;
    constexpr double shooterkP = 0.0001;
    constexpr double shooterkI = 0.0000003;
    constexpr double shooterkD = 0.0;
    constexpr double shooterkFF = 0.00002;
    constexpr double intakeBeamBreakID = 1;
    constexpr double shooterBeamBreakID = 2;
} // namespace ShooterConstants

namespace AutoConstants
{
    constexpr auto kMaxSpeed = 5.5_mps;
    // constexpr auto kMaxAcceleration = 5_mps_sq;
    constexpr auto kMaxAngularSpeed = 18_rad_per_s;
    constexpr auto kMaxAngularAcceleration = 30_rad_per_s_sq;

    constexpr double kPXController = 1.25;
    constexpr double kPYController = 1.15;
    constexpr double kPThetaController = 2;

    extern const frc::TrapezoidProfile<units::radians>::Constraints
        kThetaControllerConstraints;

} // namespace AutoConstants

namespace OIConstants
{
    constexpr int kDriverControllerPort = 0;
    constexpr int kSecondaryControllerPort = 1;
} // namespace OIConstants
