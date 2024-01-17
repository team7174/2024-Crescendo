// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <ctre/Phoenix.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/controller/SimpleMotorFeedforward.h>
#include <HardwareConfig.h>

#include "Constants.h"

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               int turningEncoderPorts, double encoderOffset);

  frc::SwerveModuleState GetState();

  frc::SwerveModulePosition GetPosition();

  double getTurnEncoderDistance();

  double getDriveEncoderRate();

  double getDriveEncoderDistance();

  void SetDesiredState(const frc::SwerveModuleState& state);

  void ResetEncoders();

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr auto kModuleMaxAngularVelocity =
      units::radians_per_second_t{std::numbers::pi};
  static constexpr auto kModuleMaxAngularAcceleration =
      units::radians_per_second_squared_t{std::numbers::pi * 2.0};

  WPI_TalonFX  m_driveMotor;
  WPI_TalonFX  m_turningMotor;
  HardwareConfig m_Settings;
  frc::AnalogEncoder m_turningEncoder;


  double absEnc;

  frc2::PIDController m_drivePIDController{
      ModuleConstants::kPModuleDriveController, 0, 0};
  frc2::PIDController steerPID{0.0075, 0.02, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      ModuleConstants::kPModuleTurningController,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
};
