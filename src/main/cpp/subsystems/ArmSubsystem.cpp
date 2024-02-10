#include "subsystems/ArmSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ArmSubsystem::ArmSubsystem(DriveSubsystem* passedDriveSubsystem)
    : m_armMotorLeft(StormbreakerConstants::leftArmID),   // Replace with your TalonFX device ID
      m_armMotorRight(StormbreakerConstants::rightArmID), // Replace with your TalonFX device ID
      m_armEncoder(StormbreakerConstants::armEncoderID),
      m_armPIDController(StormbreakerConstants::armkP, StormbreakerConstants::armkI, StormbreakerConstants::armkD)
{
  // Motor, encoder, and PID controller initialization can be done here
  // Set the PID controller's setpoint to the initial position
  m_driveSubsystem = passedDriveSubsystem;
  m_armPIDController.SetSetpoint(0.0);
  m_armPIDController.SetTolerance(1);
  m_armMotorRight.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  m_armMotorLeft.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  frc::SmartDashboard::PutNumber("Angle Offset", angleOffset);
}

void ArmSubsystem::Periodic()
{
  double speed = std::clamp(m_armPIDController.Calculate(GetAbsArmAngle()), -1.0, 1.0);
  frc::SmartDashboard::PutNumber("Through Bore Angle", GetAbsArmAngle());
  frc::SmartDashboard::PutNumber("Desired Angle", m_armPIDController.GetSetpoint());
  frc::SmartDashboard::PutNumber("Calculated Angle", CalculateAngle());
  m_armMotorLeft.Set(-speed);
  m_armMotorRight.Set(speed);
}

double ArmSubsystem::GetAbsArmAngle()
{
  return abs((m_armEncoder.GetAbsolutePosition() - StormbreakerConstants::armEncoderOffset) * 360);
}

void ArmSubsystem::SetDesiredAngle(ArmStates DesiredArmState)
{
  // Update the setpoint of the PID controller
  if (DesiredArmState == ArmStates::intake)
  {
    m_armPIDController.SetSetpoint(0);
  }
  else if (DesiredArmState == ArmStates::upright)
  {
    m_armPIDController.SetSetpoint(90);
  }
  else if (DesiredArmState == ArmStates::autoAngle)
  {
    m_armPIDController.SetSetpoint(CalculateAngle());
  }

  m_armPIDController.SetSetpoint(std::clamp(m_armPIDController.GetSetpoint(), 0.0, 100.0));
}

// void ArmSubsystem::UpdateDesiredAngleFromJoystick()
// {
//   double joystickInput = -m_armController->GetLeftY();
//   double currentAngle = m_armPIDController.GetSetpoint();

//   if (joystickInput < -0.1 || joystickInput > 0.1)
//   {
//     currentAngle = currentAngle + (joystickInput * 0.5);
//   }

//   m_armPIDController.SetSetpoint(currentAngle);
// }

double ArmSubsystem::CalculateAngle()
{
  if (auto ally = frc::DriverStation::GetAlliance())
  {
    if (ally.value() == frc::DriverStation::Alliance::kRed)
    {
      speakerX = 16.579342;
    }
    else
    {
      speakerX = 0;
    }
  }
  auto m_robotPose = m_driveSubsystem->GetPose();
  double distanceToSpeaker = sqrt(pow(speakerX - m_robotPose.X().value(), 2) + pow(5.547868 - m_robotPose.Y().value(), 2)) - 0.2032;
  double speakerHeight = 1.88;

  double shootingAngle = atan(distanceToSpeaker / speakerHeight) + asin((sin(65 * M_PI / 180) * 0.6858) / (sqrt(pow(distanceToSpeaker, 2) + pow(speakerHeight, 2)))) - (25 * M_PI / 180);

  shootingAngle = fmod(shootingAngle * (180 / M_PI), 360.0);
  shootingAngle = shootingAngle - frc::SmartDashboard::GetNumber("Angle Offset", 10);

  return shootingAngle;
}

bool ArmSubsystem::ReachedDesiredAngle()
{
  frc::SmartDashboard::PutBoolean("At Desired Angle", m_armPIDController.AtSetpoint());
  return m_armPIDController.AtSetpoint();
}

void ArmSubsystem::Stop()
{
  m_armMotorLeft.StopMotor();
  m_armMotorRight.StopMotor();
}
