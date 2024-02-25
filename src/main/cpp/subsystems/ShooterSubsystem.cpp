#include "subsystems/ShooterSubsystem.h"
#include "rev/CANSparkFlex.h"
#include <frc/smartdashboard/SmartDashboard.h>

ShooterSubsystem::ShooterSubsystem(ArmSubsystem *passedArmSubsystem, DriveSubsystem *passedDriveSubsystem)
    : m_leftShooterMotor(ShooterConstants::leftShooterID, rev::CANSparkFlex::MotorType::kBrushless),   // Replace with your TalonFX device ID
      m_rightShooterMotor(ShooterConstants::rightShooterID, rev::CANSparkFlex::MotorType::kBrushless), // Replace with your TalonFX device ID
      m_intakeMotor(ShooterConstants::intakeID, rev::CANSparkFlex::MotorType::kBrushless),             // Replace with your TalonFX device ID
      rightShooterEnc(m_rightShooterMotor.GetEncoder()),
      leftShooterEnc(m_leftShooterMotor.GetEncoder()),
      leftShooterPID(m_leftShooterMotor.GetPIDController()),
      rightShooterPID(m_rightShooterMotor.GetPIDController()),
      intakeEnc(m_intakeMotor.GetEncoder()),
      intakePID(m_intakeMotor.GetPIDController()),
      intakeBeamBreak(ShooterConstants::intakeBeamBreakID),
      shooterBeamBreak(ShooterConstants::shooterBeamBreakID)
{
  m_armSubsystem = passedArmSubsystem;
  m_drive = passedDriveSubsystem;

  m_leftShooterMotor.RestoreFactoryDefaults();
  m_rightShooterMotor.RestoreFactoryDefaults();
  m_intakeMotor.RestoreFactoryDefaults();

  // Limits
  m_leftShooterMotor.SetSmartCurrentLimit(30);
  m_rightShooterMotor.SetSmartCurrentLimit(30);
  //m_intakeMotor.SetSmartCurrentLimit(30);
  // m_leftShooterMotor.EnableVoltageCompensation(12.0);
  // m_rightShooterMotor.EnableVoltageCompensation(12.0);

  // Reset encoders
  leftShooterEnc.SetPosition(0.0);
  rightShooterEnc.SetPosition(0.0);
  intakeEnc.SetPosition(0.0);
  leftShooterEnc.SetMeasurementPeriod(10);
  rightShooterEnc.SetMeasurementPeriod(10);
  intakeEnc.SetMeasurementPeriod(10);
  leftShooterEnc.SetAverageDepth(2);
  rightShooterEnc.SetAverageDepth(2);
  intakeEnc.SetAverageDepth(2);

  // Get controllers
  setPID();

  // Disable brake mode
  m_leftShooterMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  m_rightShooterMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  m_intakeMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  m_rightShooterMotor.SetInverted(true);
  m_leftShooterMotor.SetInverted(true);

  m_leftShooterMotor.BurnFlash();
  m_rightShooterMotor.BurnFlash();
  m_intakeMotor.BurnFlash();
}

void ShooterSubsystem::Periodic()
{
  frc::SmartDashboard::PutNumber("Left Shooter Velocity", double(leftShooterEnc.GetVelocity()));
  frc::SmartDashboard::PutNumber("Intake Velocity", double(intakeEnc.GetVelocity()));
  frc::SmartDashboard::PutBoolean("AT ANGLE", m_armSubsystem->ReachedDesiredAngle());
  frc::SmartDashboard::PutBoolean("AT SPEED", ShooterAtSpeed());
  NoteInIntake();
  NoteInShooter();

  if (currIntakeState == intakeStates::intake && (NoteInIntake() || NoteInShooter()))
  {
    SetIntakeState(intakeStates::slow);
  }
  if (currIntakeState == intakeStates::slow)
  {
    if (!NoteInShooter())
    {
      intakeSpeed = 0.2;
    }
    else
    {
      intakeSpeed = 0.0;
    }
  }
  bool atShootAngle = m_drive->atShootingAngle();
  frc::SmartDashboard::PutBoolean("Aim in shoot", atShootAngle);
  if (currShooterState == shooterStates::shooterOn && atShootAngle && ShooterAtSpeed() && m_armSubsystem->ReachedDesiredAngle() && (NoteInShooter() || NoteInIntake()))
  {
    shooterTimeStamp = frc::Timer::GetFPGATimestamp();
    SetIntakeState(intakeStates::shoot);
  }

  if (currShooterState == shooterStates::shooterOn && currIntakeState == intakeStates::shoot && (frc::Timer::GetFPGATimestamp() - shooterTimeStamp) > 0.5_s)
  {
    SetIntakeState(intakeStates::stop);
    SetShooterState(shooterStates::shooterStop);
    m_armSubsystem->SetDesiredAngle(ArmSubsystem::ArmStates::intake);
  }

  if (!NoteInIntake() && currShooterState == shooterStates::shooterEject)
  {
    SetIntakeState(intakeStates::stop);
    SetShooterState(shooterStates::shooterStop);
  }
  runVelocity(shooterSpeed);
  m_intakeMotor.Set(intakeSpeed);
  //intakePID.SetReference(intakeSpeed, rev::CANSparkBase::ControlType::kVelocity);
}

void ShooterSubsystem::SetIntakeState(intakeStates intakeState)
{
  currIntakeState = intakeState;
  switch (intakeState)
  {
  case intakeStates::intake:
    intakeSpeed = 1;
    break;

  case intakeStates::shoot:
    intakeSpeed = 1;
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
    shooterSpeed = 5000;
    break;

  case shooterStates::shooterStop:
    if (frc::DriverStation::IsAutonomous())
    {
      shooterSpeed = 5000;
    }
    else
    {
      shooterSpeed = 2000;
    }
    break;

  case shooterStates::shooterEject:
    shooterSpeed = -3000;
    break;

  default:
    shooterSpeed = 0.0;
  }
}

bool ShooterSubsystem::NoteInIntake()
{
  frc::SmartDashboard::PutBoolean("Intake Beam Break", !intakeBeamBreak.Get());
  return (!intakeBeamBreak.Get());
}

bool ShooterSubsystem::NoteInShooter()
{
  frc::SmartDashboard::PutBoolean("Shooter Beam Break", !shooterBeamBreak.Get());
  return (!shooterBeamBreak.Get());
}

bool ShooterSubsystem::NoteInBoth()
{
  return (NoteInIntake() && NoteInShooter());
}

bool ShooterSubsystem::ShooterAtSpeed()
{
  return (leftShooterEnc.GetVelocity() > (shooterSpeed - 150) && rightShooterEnc.GetVelocity() > (shooterSpeed - 150));
}

void ShooterSubsystem::runVelocity(double rpm)
{
  leftShooterPID.SetReference(rpm, rev::CANSparkBase::ControlType::kVelocity);
  rightShooterPID.SetReference(rpm, rev::CANSparkBase::ControlType::kVelocity);
}

void ShooterSubsystem::setPID()
{
  leftShooterPID.SetP(ShooterConstants::shooterkP);
  leftShooterPID.SetI(ShooterConstants::shooterkI);
  leftShooterPID.SetD(ShooterConstants::shooterkD);

  rightShooterPID.SetP(ShooterConstants::shooterkP);
  rightShooterPID.SetI(ShooterConstants::shooterkI);
  rightShooterPID.SetD(ShooterConstants::shooterkD);

  intakePID.SetP(ShooterConstants::intakekP);
  intakePID.SetI(ShooterConstants::intakekI);
  intakePID.SetD(ShooterConstants::intakekD);

  leftShooterPID.SetFF(ShooterConstants::shooterkFF);
  rightShooterPID.SetFF(ShooterConstants::shooterkFF);
  intakePID.SetFF(ShooterConstants::intakekFF);
}

void ShooterSubsystem::Stop()
{
  m_leftShooterMotor.StopMotor();
  m_rightShooterMotor.StopMotor();
}
