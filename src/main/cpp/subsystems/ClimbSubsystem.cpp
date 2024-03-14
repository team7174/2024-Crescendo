#include "subsystems/ClimbSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

ClimbSubsystem::ClimbSubsystem()
    : m_climbMotorLeft(ClimberConstants::leftClimbID),  // Replace with your TalonFX device ID
      m_climbMotorRight(ClimberConstants::rightClimbID),
      m_climbPIDController(ClimberConstants::climbkP, ClimberConstants::climbkI, ClimberConstants::climbkD) {
  m_climbMotorRight.SetInverted(true);

  m_climbMotorLeft.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  m_climbMotorRight.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void ClimbSubsystem::Periodic() {
  m_climbMotorLeft.Set(std::clamp(m_climbPIDController.Calculate(m_climbMotorLeft.GetPosition().GetValueAsDouble()), -1.0, 1.0));
  m_climbMotorRight.Set(std::clamp(m_climbPIDController.Calculate(m_climbMotorRight.GetPosition().GetValueAsDouble()), -1.0, 1.0));
}

void ClimbSubsystem::SetClimbState(ClimbStates DesiredClimbState) {
  switch (DesiredClimbState) {
    case ClimbStates::extend:
      m_climbPIDController.SetSetpoint(110.0);
      break;

    case ClimbStates::hold:
      m_climbPIDController.SetSetpoint((m_climbMotorLeft.GetPosition().GetValueAsDouble() + m_climbMotorRight.GetPosition().GetValueAsDouble()) / 2);
      break;

    default:
      m_climbPIDController.SetSetpoint(0);
      break;
  }

  m_climbPIDController.SetSetpoint(std::clamp(m_climbPIDController.GetSetpoint(), 0.0, 110.0));
}

void ClimbSubsystem::Stop() {
  m_climbMotorLeft.StopMotor();
  m_climbMotorRight.StopMotor();
}
