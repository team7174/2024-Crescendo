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
      m_turningEncoder(turningEncoderPorts)
{

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningEncoder.SetPositionOffset(encoderOffset);
  steerPID.EnableContinuousInput(-180, 180);

  m_driveMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});
  m_driveMotor.GetConfigurator().Apply(motorConfigs.DriveMotorConfig, 50_ms);
  m_driveMotor.SetInverted(false);
  m_driveMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  m_driveMotor.SetPosition(units::angle::turn_t(0));

  m_turningMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});
  m_turningMotor.GetConfigurator().Apply(motorConfigs.TurnMotorConfig, 50_ms);
  m_turningMotor.GetConfigurator().Apply(motorConfigs.TurnCurrLimit, 50_ms);
  m_turningMotor.GetConfigurator().Apply(motorConfigs.TurnVoltageLimit, 50_ms);
  m_turningMotor.SetInverted(false);
  m_turningMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);

  m_driveMotor.GetVelocity().SetUpdateFrequency(50_Hz);
  m_turningMotor.GetVelocity().SetUpdateFrequency(50_Hz);

  m_driveMotor.GetPosition().SetUpdateFrequency(50_Hz);
  m_turningMotor.GetPosition().SetUpdateFrequency(50_Hz);
}

double SwerveModule::getTurnEncoderDistance()
{
  absEnc = ((m_turningEncoder.GetAbsolutePosition() - m_turningEncoder.GetPositionOffset()) * 360);
  if (absEnc < 0)
  {
    absEnc += 360.0;
  }
  if (absEnc > 180)
  {
    absEnc = -(360 - absEnc);
  }
  return absEnc;
}

double SwerveModule::getDriveEncoderRate()
{
  // return m_driveMotor.getvelocity
  return m_driveMotor.GetVelocity().GetValueAsDouble() / ModuleConstants::driveGearRatio * ModuleConstants::kWheelCircumference;
  // return ((m_driveMotor.GetVelocity().GetValueAsDouble() * ModuleConstants::kWheelCircumference) / (ModuleConstants::driveEncoderCPR));
}

double SwerveModule::getDriveEncoderDistance()
{
  return m_driveMotor.GetPosition().GetValueAsDouble() / ModuleConstants::driveGearRatio * ModuleConstants::kWheelCircumference;

  // return ((m_driveMotor.GetPosition().GetValueAsDouble() * ModuleConstants::kWheelCircumference) / (ModuleConstants::driveEncoderCPR));
}

frc::SwerveModuleState SwerveModule::GetState()
{
  return {units::meters_per_second_t{getDriveEncoderRate()},
          units::degree_t{getTurnEncoderDistance()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition()
{
  return {units::meter_t{getDriveEncoderDistance()},
          units::degree_t{getTurnEncoderDistance()}};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState &referenceState)
{
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(referenceState, units::degree_t{getTurnEncoderDistance()});

  units::turns_per_second_t targetMotorSpeed((state.speed.value() / ModuleConstants::kWheelCircumference) * ModuleConstants::driveGearRatio);

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = steerPID.Calculate(getTurnEncoderDistance(), double{state.angle.Degrees()});

  frc::SmartDashboard::PutNumber(std::to_string(m_driveMotor.GetDeviceID()) + "Desired Speed", double{state.speed.value()});
  frc::SmartDashboard::PutNumber(std::to_string(m_driveMotor.GetDeviceID()) + "Curr Speed", getDriveEncoderRate());
  frc::SmartDashboard::PutNumber(std::to_string(m_driveMotor.GetDeviceID()) + "Desired Motor Speed", targetMotorSpeed.value());
  frc::SmartDashboard::PutNumber(std::to_string(m_driveMotor.GetDeviceID()) + "Curr Motor Speed", m_driveMotor.GetVelocity().GetValueAsDouble());

  frc::SmartDashboard::PutNumber(std::to_string(m_turningMotor.GetDeviceID()) + "Desired Pos", double{state.angle.Degrees()});
  frc::SmartDashboard::PutNumber(std::to_string(m_turningMotor.GetDeviceID()) + "Curr Pos", getTurnEncoderDistance());

  m_turningMotor.Set(turnOutput);
  m_driveMotor.SetControl(m_driveRequest.WithVelocity(targetMotorSpeed));
}

void SwerveModule::ResetEncoders()
{
  m_driveMotor.SetPosition(units::angle::turn_t(0));
}
