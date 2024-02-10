#include "subsystems/ShooterSubsystem.h"
#include "rev/CANSparkFlex.h"
#include <frc/smartdashboard/SmartDashboard.h>

ShooterSubsystem::ShooterSubsystem(ArmSubsystem *passedArmSubsystem)
    : m_leftShooterMotor(ShooterConstants::leftShooterID, rev::CANSparkFlex::MotorType::kBrushless),   // Replace with your TalonFX device ID
      m_rightShooterMotor(ShooterConstants::rightShooterID, rev::CANSparkFlex::MotorType::kBrushless), // Replace with your TalonFX device ID
      m_intakeMotor(ShooterConstants::intakeID, rev::CANSparkFlex::MotorType::kBrushless),             // Replace with your TalonFX device ID
      rightShooterEnc(m_rightShooterMotor.GetEncoder()),
      leftShooterEnc(m_leftShooterMotor.GetEncoder())
// m_vortexPID(ShooterConstants::shooterkP, ShooterConstants::shooterkI, ShooterConstants::shooterkD),
{
  // Motor, encoder, and PID controller initialization can be done here
  // Set the PID controller's setpoint to the initial position
  m_rightShooterMotor.SetInverted(true);
  m_leftShooterMotor.SetInverted(true);
  m_armSubsystem = passedArmSubsystem;
}

void ShooterSubsystem::Periodic()
{
  frc::SmartDashboard::PutNumber("Left Shooter Velocity", double(leftShooterEnc.GetVelocity()));

  if (currIntakeState == intakeStates::intake && NotePresent())
  {
    SetIntakeState(intakeStates::stop);
  }

  if (currShooterState == shooterStates::shooterOn && ShooterAtSpeed() && m_armSubsystem->ReachedDesiredAngle())
  {
    SetIntakeState(intakeStates::shoot);
  }

  if (!NotePresent() && currShooterState == shooterStates::shooterOn)
  {
    SetIntakeState(intakeStates::stop);
    SetShooterState(shooterStates::shooterStop);
    m_armSubsystem->SetDesiredAngle(ArmSubsystem::ArmStates::intake);
  }

  if (!NotePresent() && currShooterState == shooterStates::shooterEject)
  {
    SetIntakeState(intakeStates::stop);
    SetShooterState(shooterStates::shooterStop);
  }

  frc::SmartDashboard::PutNumber("Shooter State", currShooterState);
  frc::SmartDashboard::PutNumber("Intake State", currIntakeState);

  m_leftShooterMotor.Set(shooterSpeed);
  m_rightShooterMotor.Set(shooterSpeed);
  m_intakeMotor.Set(intakeSpeed);
}

void ShooterSubsystem::SetIntakeState(intakeStates intakeState)
{
  currIntakeState = intakeState;
  switch (intakeState)
  {
  case intakeStates::intake:
    intakeSpeed = 0.5;
    break;

  case intakeStates::shoot:
    intakeSpeed = 1.0;
    break;

  case intakeStates::eject:
    intakeSpeed = -0.5;
    break;

  case intakeStates::stop:
    intakeSpeed = 0;
    break;

  default:
    intakeSpeed = 0.0;
  }
}

void ShooterSubsystem::SetShooterState(shooterStates shooterState)
{
  currShooterState = shooterState;
  switch (shooterState)
  {
  case shooterStates::shooterOn:
    shooterSpeed = 0.6;
    break;

  case shooterStates::shooterStop:
    shooterSpeed = 0;
    break;

  case shooterStates::shooterEject:
    shooterSpeed = -0.5;
    break;

  default:
    shooterSpeed = 0.0;
  }
}

bool ShooterSubsystem::NotePresent()
{
  frc::SmartDashboard::PutBoolean("Shooter Beam Break", !shooterBeamBreak.Get());
  frc::SmartDashboard::PutBoolean("Intake Beam Break", !intakeBeamBreak.Get());
  return (!intakeBeamBreak.Get() || !shooterBeamBreak.Get());
}

bool ShooterSubsystem::ShooterAtSpeed()
{
  return (leftShooterEnc.GetVelocity() > (shooterSpeed * 6500) && rightShooterEnc.GetVelocity() > (shooterSpeed * 6500));
}

void ShooterSubsystem::Stop()
{
  m_leftShooterMotor.StopMotor();
  m_rightShooterMotor.StopMotor();
}
