#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DutyCycleEncoder.h>
#include <ctre/Phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include "Constants.h"
#include <frc/XboxController.h>
#include <frc/DriverStation.h>

class ArmSubsystem : public frc2::SubsystemBase
{
public:
  ArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  enum ArmStates
  {
    intake,
    autoAngle,
    upright
  };
  void SetDesiredAngle(ArmStates DesiredArmState);
  void Stop();
  double GetAbsArmAngle();
  void UpdateDesiredAngleFromJoystick();
  double CalculateAngle();
  bool ReachedDesiredAngle();

private:
  ctre::phoenix6::hardware::TalonFX m_armMotorLeft;
  ctre::phoenix6::hardware::TalonFX m_armMotorRight;
  frc::DutyCycleEncoder m_armEncoder;
  frc::PIDController m_armPIDController;
  // DriveSubsystem m_driveSubsystem;
  double speakerX;
};
