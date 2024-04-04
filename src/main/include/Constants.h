// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

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

#include <numbers>

#pragma once

namespace DriveConstants {
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

constexpr double kFrontLeftEncoderOffset = 0.262388378327935;
constexpr double kRearLeftEncoderOffset = 0.351059220727892;
constexpr double kFrontRightEncoderOffset = 0.843946522195772;
constexpr double kRearRightEncoderOffset = 0.9165161825489496;
constexpr bool flipFL = false;
constexpr bool flipFR = false;
constexpr bool flipBL = false;
constexpr bool flipBR = false;

constexpr double frontWheelOffset = 0.1016;

}  // namespace DriveConstants

namespace ModuleConstants {
constexpr double kWheelDiameterMeters = 0.1016;
constexpr double kWheelCircumference = kWheelDiameterMeters * std::numbers::pi;
constexpr double driveGearRatio = 6.12;
constexpr double angleGearRatio = 12.8;

constexpr double kPModuleTurningController = 1;
constexpr double kPModuleDriveController = 1;
}  // namespace ModuleConstants

namespace StormbreakerConstants {
constexpr int armEncoderID = 8;
constexpr int leftArmID = 50;
constexpr int rightArmID = 51;
constexpr double armEncoderOffset = 0.297579907439498;
constexpr double armkP = 0.0225;
constexpr double armkI = 0.0;
constexpr double armkD = 0.0;
constexpr double armLength = 0.42209075097556;
constexpr double shooterDefaultAngle = 65.0;
constexpr double armToRobotAngle = 21.70;
constexpr double shooterToArmAngle = 43.30;
constexpr double pivotHeight = 0.4826;
constexpr double pivotBack = 0.2794;
constexpr auto kArmAngleSpeed = 15_deg_per_s;
constexpr auto kArmAngleAcceleration = 10_deg_per_s_sq;
}  // namespace StormbreakerConstants

namespace ClimberConstants {
constexpr int leftClimbID = 60;
constexpr int rightClimbID = 61;
constexpr double climbkP = 0.025;
constexpr double climbkI = 0.0;
constexpr double climbkD = 0.0;
}  // namespace ClimberConstants

namespace ShooterConstants {
constexpr int leftShooterID = 55;
constexpr int rightShooterID = 56;
constexpr int intakeID = 57;
constexpr int rollerID = 58;
constexpr double shooterkP = 0.0001;
constexpr double shooterkI = 0.0000003;
constexpr double shooterkD = 0.0;
constexpr double shooterkFF = 0.00002;
constexpr double intakekP = 0.0005;
constexpr double intakekI = 0.0000003;
constexpr double intakekD = 0.0;
constexpr double intakekFF = 0.0001;
constexpr int intakeBeamBreakID = 1;
constexpr int shooterBeamBreakID = 2;
}  // namespace ShooterConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 5.5_mps;
// constexpr auto kMaxAcceleration = 5_mps_sq;
constexpr auto kMaxAngularSpeed = 15_rad_per_s;
constexpr auto kMaxAngularAcceleration = 30_rad_per_s_sq;

constexpr double kPXController = 8.0;
constexpr double kPYController = 1.15;
constexpr double kPThetaController = 4.0;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace DriveConstants {
constexpr auto kMaxSpeed = 5.0_mps;
constexpr auto kMaxAngularSpeed = 6_rad_per_s;
constexpr auto kMaxAngularAcceleration = 5_rad_per_s_sq;
}  // namespace DriveConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kSecondaryControllerPort = 1;
}  // namespace OIConstants