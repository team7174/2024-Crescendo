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

    constexpr double kFrontLeftEncoderOffset = 0.82260926218418;
    constexpr double kRearLeftEncoderOffset = 0.315686777968225;
    constexpr double kFrontRightEncoderOffset = 0.145569385478036;
    constexpr double kRearRightEncoderOffset = 0.505396518062415;
} // namespace DriveConstants

namespace ModuleConstants
{
    constexpr double encToAngle = 0.0146484375;
    constexpr double kWheelDiameterMeters = 0.1016;
    constexpr double kWheelCircumference = kWheelDiameterMeters * std::numbers::pi;
    constexpr double driveGearRatio = 6;
    constexpr double turnGearRatio = 12;
    constexpr int driveEncoderCPR = 2048*driveGearRatio;
    constexpr int turnEncoderCPR = 2048*turnGearRatio;

    constexpr double kPModuleTurningController = 1;
    constexpr double kPModuleDriveController = 1;
} // namespace ModuleConstants

namespace AutoConstants
{
    constexpr auto kMaxSpeed = 1_mps;
    constexpr auto kMaxAcceleration = 1_mps_sq;
    constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
    constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

    constexpr double kPXController = 1;
    constexpr double kPYController = 1;
    constexpr double kPThetaController = 2;

    extern const frc::TrapezoidProfile<units::radians>::Constraints
        kThetaControllerConstraints;

} // namespace AutoConstants

namespace OIConstants
{
    constexpr int kDriverControllerPort = 0;
    constexpr int kSecondaryControllerPort = 1;
} // namespace OIConstants
