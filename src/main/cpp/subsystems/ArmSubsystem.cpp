#include "subsystems/ArmSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ArmSubsystem::ArmSubsystem(frc::XboxController *m_secondaryController)
    : m_armMotorLeft(StormbreakerConstants::leftArmID),   // Replace with your TalonFX device ID
      m_armMotorRight(StormbreakerConstants::rightArmID), // Replace with your TalonFX device ID
      m_armEncoder(StormbreakerConstants::armEncoderID),
      m_armPIDController(StormbreakerConstants::armkP, StormbreakerConstants::armkI, StormbreakerConstants::armkD),
      m_armController(m_secondaryController)
{
  // Motor, encoder, and PID controller initialization can be done here
  // Set the PID controller's setpoint to the initial position
  m_armPIDController.SetSetpoint(0.0);
}

void ArmSubsystem::Periodic()
{
  double speed = std::clamp(m_armPIDController.Calculate(GetAbsArmAngle()), -1.0, 1.0);
  frc::SmartDashboard::PutNumber("Through Bore Angle", GetAbsArmAngle());
  frc::SmartDashboard::PutNumber("Arm Speed", speed);
  frc::SmartDashboard::PutNumber("Desired Angle", m_armPIDController.GetSetpoint());
  m_armMotorLeft.Set(-speed);
  m_armMotorRight.Set(speed);
}

double ArmSubsystem::GetAbsArmAngle()
{
  return abs((m_armEncoder.GetAbsolutePosition() - StormbreakerConstants::armEncoderOffset) * 360);
}

void ArmSubsystem::SetDesiredAngle(frc::XboxController *m_secondaryController)
{
  // Update the setpoint of the PID controller
  if (m_secondaryController->GetXButton())
  {
    m_armPIDController.SetSetpoint(0);
  }
  else if (m_secondaryController->GetBButton())
  {
    m_armPIDController.SetSetpoint(90);
  }
  else
  {
    UpdateDesiredAngleFromJoystick();
  }
  if (m_armPIDController.GetSetpoint() < 0)
  {
    m_armPIDController.SetSetpoint(0);
  }
  if (m_armPIDController.GetSetpoint() > 100)
  {
    m_armPIDController.SetSetpoint(100);
  }
}

void ArmSubsystem::UpdateDesiredAngleFromJoystick()
{
  double joystickInput = -m_armController->GetLeftY();
  double currentAngle = m_armPIDController.GetSetpoint();

  if (joystickInput < -0.1 || joystickInput > 0.1)
  {
    currentAngle = currentAngle + (joystickInput * 0.5);
  }

  m_armPIDController.SetSetpoint(currentAngle);
}

// double ArmSubsystem::CalculateAngle() {
//     double distanceToSpeaker = sqrt(pow(m_robotPose->X().value(), 2) + pow(5.547868 - m_robotPose->Y().value(), 2)) - 0.2032;
//     double speakerHeight = 1.8288;

//     double shootingAngle = atan(distanceToSpeaker / speakerHeight) + asin((sin(65*M_PI/180) * 0.70485) / (sqrt(pow(distanceToSpeaker, 2) + pow(speakerHeight, 2)))) - (25 * M_PI/180);

//     shootingAngle = fmod(shootingAngle * (180 / M_PI), 360.0);

//     return shootingAngle;
// }

void ArmSubsystem::Stop()
{
  m_armMotorLeft.StopMotor();
  m_armMotorRight.StopMotor();
}
