#include "subsystems/ShooterSubsystem.h"
#include "rev/CANSparkFlex.h"
#include <frc/smartdashboard/SmartDashboard.h>

ShooterSubsystem::ShooterSubsystem(ArmSubsystem *passedArmSubsystem)
    : m_leftShooterMotor(ShooterConstants::leftShooterID, rev::CANSparkFlex::MotorType::kBrushless),   // Replace with your TalonFX device ID
      m_rightShooterMotor(ShooterConstants::rightShooterID, rev::CANSparkFlex::MotorType::kBrushless), // Replace with your TalonFX device ID
      m_intakeMotor(ShooterConstants::intakeID, rev::CANSparkFlex::MotorType::kBrushless),             // Replace with your TalonFX device ID
      rightShooterEnc(m_rightShooterMotor.GetEncoder()),
      leftShooterEnc(m_leftShooterMotor.GetEncoder()),
      leftShooterPID(m_leftShooterMotor.GetPIDController()),
      rightShooterPID(m_rightShooterMotor.GetPIDController())
{
  // Motor, encoder, and PID controller initialization can be done here
  // Set the PID controller's setpoint to the initial position
  m_armSubsystem = passedArmSubsystem;

  m_leftShooterMotor.RestoreFactoryDefaults();
  m_rightShooterMotor.RestoreFactoryDefaults();

  // Limits
  m_leftShooterMotor.SetSmartCurrentLimit(60);
  m_rightShooterMotor.SetSmartCurrentLimit(60);
  m_leftShooterMotor.EnableVoltageCompensation(12.0);
  m_rightShooterMotor.EnableVoltageCompensation(12.0);

  // Reset encoders
  leftShooterEnc.SetPosition(0.0);
  rightShooterEnc.SetPosition(0.0);
  leftShooterEnc.SetMeasurementPeriod(10);
  rightShooterEnc.SetMeasurementPeriod(10);
  leftShooterEnc.SetAverageDepth(2);
  rightShooterEnc.SetAverageDepth(2);

  // Get controllers
  setPID();

  // Disable brake mode
  m_leftShooterMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  m_rightShooterMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  m_rightShooterMotor.SetInverted(true);
  m_leftShooterMotor.SetInverted(true);

  m_leftShooterMotor.BurnFlash();
  m_rightShooterMotor.BurnFlash();
}

void ShooterSubsystem::Periodic()
{
  frc::SmartDashboard::PutNumber("Left Shooter Velocity", double(leftShooterEnc.GetVelocity()));

  if (currIntakeState == intakeStates::intake && NotePresent())
  {
    SetIntakeState(intakeStates::stop);
  }
  frc::SmartDashboard::PutBoolean("AT ANGLE", m_armSubsystem->ReachedDesiredAngle());
  frc::SmartDashboard::PutBoolean("AT SPEED", ShooterAtSpeed());
  if (currShooterState == shooterStates::shooterOn && ShooterAtSpeed() && m_armSubsystem->ReachedDesiredAngle())
  {
    currentTime = frc::Timer::GetFPGATimestamp();
    SetIntakeState(intakeStates::shoot);
  }
  if (!NotePresent() && currShooterState == shooterStates::shooterOn && currIntakeState == intakeStates::shoot && (frc::Timer::GetFPGATimestamp() - currentTime) > 5_s)
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

  runVelocity(shooterSpeed);
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
    shooterSpeed = 6500;
    break;

  case shooterStates::shooterStop:
    shooterSpeed = 2000;
    break;

  case shooterStates::shooterEject:
    shooterSpeed = -3000;
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
  return (leftShooterEnc.GetVelocity() > (shooterSpeed - 100) && rightShooterEnc.GetVelocity() > (shooterSpeed - 100));
}

void ShooterSubsystem::runVelocity(double rpm)
{
  leftShooterPID.SetReference(rpm, rev::CANSparkBase::ControlType::kVelocity);
  rightShooterPID.SetReference(rpm, rev::CANSparkBase::ControlType::kVelocity);
}

void ShooterSubsystem::setPID()
{
  leftShooterPID.SetP(kP);
  leftShooterPID.SetI(kI);
  leftShooterPID.SetD(kD);

  rightShooterPID.SetP(kP);
  rightShooterPID.SetI(kI);
  rightShooterPID.SetD(kD);

  leftShooterPID.SetFF(kff);
  rightShooterPID.SetFF(kff);
}

void ShooterSubsystem::Stop()
{
  m_leftShooterMotor.StopMotor();
  m_rightShooterMotor.StopMotor();
}
