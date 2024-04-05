#pragma once

#include <frc/DigitalInput.h>
#include <frc/DriverStation.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>
#include <subsystems/DriveSubsystem.h>

#include <ctre/Phoenix6/TalonFX.hpp>

#include "Constants.h"

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem(DriveSubsystem *);
  DriveSubsystem *m_driveSubsystem;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  enum ArmStates {
    intake,
    autoAngle,
    upright
  };
  ArmStates currArmState;
  void SetDesiredAngle(ArmStates DesiredArmState);
  void Stop();
  double GetAbsArmAngle();
  double CalculateAngle();
  bool ReachedDesiredAngle();
  void brakeModeOff();
  void brakeModeOn();
  static constexpr units::degrees_per_second_t kMaxAngularSpeed = 360.0_deg_per_s;
  static constexpr units::degrees_per_second_squared_t kMaxAngularAcceleration = 720.0_deg_per_s_sq;

 private:
  ctre::phoenix6::hardware::TalonFX m_armMotorLeft;
  ctre::phoenix6::hardware::TalonFX m_armMotorRight;
  frc::DutyCycleEncoder m_armEncoder;
  frc::ProfiledPIDController<units::degrees> profiledController;
  double angleOffset = 2.5;
  frc::DigitalInput armSwitch{5};
  bool brakeMode = true;
};