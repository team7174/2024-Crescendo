#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DutyCycleEncoder.h>
#include <ctre/Phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include "Constants.h"
#include <frc/XboxController.h>

class ArmSubsystem : public frc2::SubsystemBase
{
public:
  ArmSubsystem(frc::XboxController *m_secondaryController);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SetDesiredAngle(frc::XboxController *m_secondaryController);
  void Stop();
  double GetAbsArmAngle();
  void UpdateDesiredAngleFromJoystick();
  double CalculateAngle();

private:
  ctre::phoenix6::hardware::TalonFX m_armMotorLeft;
  ctre::phoenix6::hardware::TalonFX m_armMotorRight;
  frc::DutyCycleEncoder m_armEncoder;
  frc::PIDController m_armPIDController;
  frc::XboxController *m_armController;
  frc::Pose2d *m_robotPose;
};
