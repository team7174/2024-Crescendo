#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DutyCycleEncoder.h>
#include <ctre/Phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include "Constants.h"
#include <frc/XboxController.h>
#include <frc/DriverStation.h>
#include <subsystems/DriveSubsystem.h>
#include <frc/DigitalInput.h>

class ArmSubsystem : public frc2::SubsystemBase
{
public:
  ArmSubsystem(DriveSubsystem *);
  DriveSubsystem *m_driveSubsystem;

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
  ArmStates currArmState;
  void SetDesiredAngle(ArmStates DesiredArmState);
  void Stop();
  double GetAbsArmAngle();
  void UpdateDesiredAngleFromJoystick();
  double CalculateAngle();
  bool ReachedDesiredAngle();
  void brakeModeOff();
  void brakeModeOn();
  static constexpr units::degrees_per_second_t kMaxAngularSpeed = 180.0_deg_per_s;
  static constexpr units::degrees_per_second_squared_t kMaxAngularAcceleration = 360.0_deg_per_s_sq;

private:
  ctre::phoenix6::hardware::TalonFX m_armMotorLeft;
  ctre::phoenix6::hardware::TalonFX m_armMotorRight;
  frc::DutyCycleEncoder m_armEncoder;
  frc::ProfiledPIDController<units::degrees> profiledController;
  double angleOffset = 0;
  frc::DigitalInput armSwitch{5};
  bool brakeMode = true;
};
