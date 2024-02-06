#include "subsystems/ClimbSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ClimbSubsystem::ClimbSubsystem(frc::XboxController *m_secondaryController)
    : m_climbMotorLeft(ClimberConstants::leftClimbID), // Replace with your TalonFX device ID
      m_climbMotorRight(ClimberConstants::rightClimbID),
      m_climbPIDController(ClimberConstants::climbkP, ClimberConstants::climbkI, ClimberConstants::climbkD),
      m_armController(m_secondaryController)
{
}

void ClimbSubsystem::Periodic()
{
    if (climbUp)
    {
        m_climbMotorLeft.Set(std::clamp(m_climbPIDController.Calculate(m_climbMotorLeft.GetPosition().GetValueAsDouble(), 825.0), -1.0, 1.0));
        m_climbMotorRight.Set(std::clamp(m_climbPIDController.Calculate(m_climbMotorRight.GetPosition().GetValueAsDouble(), -825.0), -1.0, 1.0));
    }
    else
    {
        m_climbMotorLeft.Set(std::clamp(m_climbPIDController.Calculate(m_climbMotorLeft.GetPosition().GetValueAsDouble(), 0), -1.0, 1.0));
        m_climbMotorRight.Set(std::clamp(m_climbPIDController.Calculate(m_climbMotorRight.GetPosition().GetValueAsDouble(), 0), -1.0, 1.0));
    }
}

void ClimbSubsystem::SetDesiredPosition(frc::XboxController *m_secondaryController)
{
    // Update the setpoint of the PID controller
    if (m_secondaryController->GetYButton())
    {
        climbUp = true;
    }
    else if (m_secondaryController->GetAButton())
    {
        climbUp = false;
    }
    if (m_secondaryController->GetRightBumper())
    {
        m_climbMotorLeft.SetPosition(units::angle::turn_t(0));
        m_climbMotorRight.SetPosition(units::angle::turn_t(0));
    }
}

void ClimbSubsystem::Stop()
{
    m_climbMotorLeft.StopMotor();
    m_climbMotorRight.StopMotor();
}
