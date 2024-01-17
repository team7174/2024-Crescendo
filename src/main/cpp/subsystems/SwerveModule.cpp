// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
                           int turningEncoderPorts, double encoderOffset)
    : m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel),
      m_turningEncoder(turningEncoderPorts) {

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningEncoder.SetPositionOffset(encoderOffset);
  steerPID.EnableContinuousInput(-180, 180);

  m_driveMotor.ConfigFactoryDefault();
  m_driveMotor.ConfigAllSettings(m_Settings.DriveMotorConfig);
  m_driveMotor.SetInverted(false);  
  m_driveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_driveMotor.SetSelectedSensorPosition(0);
  m_driveMotor.EnableVoltageCompensation(true);

  m_turningMotor.ConfigFactoryDefault();
  m_turningMotor.ConfigAllSettings(m_Settings.TurnMotorConfig);
  m_turningMotor.SetInverted(true);
  m_turningMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

double SwerveModule::getTurnEncoderDistance() {
  absEnc = ((m_turningEncoder.GetAbsolutePosition() - m_turningEncoder.GetPositionOffset()) * 360);
  if (absEnc < 0) {
        absEnc += 360.0;
    }
    if(absEnc > 180) {
        absEnc = -(360 - absEnc);
    }
  return absEnc;
}

double SwerveModule::getDriveEncoderRate() {
  return ((m_driveMotor.GetSelectedSensorVelocity(0) * 10 * ModuleConstants::kWheelCircumference) / (ModuleConstants::driveEncoderCPR));
}

double SwerveModule::getDriveEncoderDistance() {
    return ((m_driveMotor.GetSelectedSensorPosition(0) * ModuleConstants::kWheelCircumference) / (ModuleConstants::driveEncoderCPR));
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{getDriveEncoderRate()},
          units::degree_t{getTurnEncoderDistance()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {units::meter_t{getDriveEncoderDistance()},
          units::degree_t{getTurnEncoderDistance()}};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(referenceState, units::degree_t{getTurnEncoderDistance()});

  const auto targetMotorSpeed = (state.speed.value() * ModuleConstants::driveEncoderCPR / (ModuleConstants::kWheelCircumference * 10));

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = steerPID.Calculate(getTurnEncoderDistance(), double{state.angle.Degrees()});

  frc::SmartDashboard::PutNumber(std::to_string(m_driveMotor.GetDeviceID()) + "Desired Speed", double{state.speed.value()});
  frc::SmartDashboard::PutNumber(std::to_string(m_driveMotor.GetDeviceID()) + "Curr Speed", getDriveEncoderRate());
  frc::SmartDashboard::PutNumber(std::to_string(m_driveMotor.GetDeviceID()) + "Desired Velocity", targetMotorSpeed);
  frc::SmartDashboard::PutNumber(std::to_string(m_turningMotor.GetDeviceID()) + "Curr Velocity", m_driveMotor.GetSelectedSensorVelocity(0));
  // Set the motor outputs.
  //m_turningMotor.Set(0);
  //m_driveMotor.Set(0);

  m_turningMotor.Set(turnOutput);
  m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, targetMotorSpeed);
}

void SwerveModule::ResetEncoders() {
  m_driveMotor.SetSelectedSensorPosition(0);
}
