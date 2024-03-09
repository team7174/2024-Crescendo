#include "subsystems/ShooterSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/CANSparkFlex.h"

ShooterSubsystem::ShooterSubsystem(ArmSubsystem *passedArmSubsystem, DriveSubsystem *passedDriveSubsystem, VisionSubsystem *passedVisionSubsystem, frc::XboxController *passedSecondaryController, frc::XboxController *passedDriveController)
    : m_leftShooterMotor(ShooterConstants::leftShooterID, rev::CANSparkFlex::MotorType::kBrushless),    // Replace with your TalonFX device ID
      m_rightShooterMotor(ShooterConstants::rightShooterID, rev::CANSparkFlex::MotorType::kBrushless),  // Replace with your TalonFX device ID
      m_intakeMotor(ShooterConstants::intakeID, rev::CANSparkFlex::MotorType::kBrushless),              // Replace with your TalonFX device ID
      rightShooterEnc(m_rightShooterMotor.GetEncoder()),
      leftShooterEnc(m_leftShooterMotor.GetEncoder()),
      leftShooterPID(m_leftShooterMotor.GetPIDController()),
      rightShooterPID(m_rightShooterMotor.GetPIDController()),
      intakeEnc(m_intakeMotor.GetEncoder()),
      intakePID(m_intakeMotor.GetPIDController()) {
  m_armSubsystem = passedArmSubsystem;
  m_drive = passedDriveSubsystem;
  m_secondaryController = passedSecondaryController;
  m_driveController = passedDriveController;
  m_visionSubsystem = passedVisionSubsystem;

  m_leftShooterMotor.RestoreFactoryDefaults();
  m_rightShooterMotor.RestoreFactoryDefaults();
  m_intakeMotor.RestoreFactoryDefaults();

  // Limits
  m_leftShooterMotor.SetSmartCurrentLimit(30);
  m_rightShooterMotor.SetSmartCurrentLimit(30);
  // m_intakeMotor.SetSmartCurrentLimit(30);
  //  m_leftShooterMotor.EnableVoltageCompensation(12.0);
  //  m_rightShooterMotor.EnableVoltageCompensation(12.0);

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

  frc::SmartDashboard::PutNumber("Shooter Velocity Threshold", 150.0);
  frc::SmartDashboard::PutNumber("Shooter Velocity", 6500.0);
  shooterIdleChooser.SetDefaultOption("On", "On");
  shooterIdleChooser.AddOption("Off", "Off");
  frc::SmartDashboard::PutData("Ignore Beam Break", &shooterIdleChooser);
}

void ShooterSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Left Shooter Velocity", double(leftShooterEnc.GetVelocity()));
  frc::SmartDashboard::PutBoolean("AT ANGLE", m_armSubsystem->ReachedDesiredAngle());
  frc::SmartDashboard::PutBoolean("AT SPEED", ShooterAtSpeed());
  NoteInIntake();
  NoteInShooter();

  // if (currIntakeState == intakeStates::intake) {
  //   if (shooterIdleChooser.GetSelected() == "On") {
  //     shooterSpeed = 2000;
  //   } else {
  //     shooterSpeed = 0;
  //   }
  // }

  if (currIntakeState == intakeStates::intake && (NoteInIntake() || NoteInShooter())) {
    intakeTimeStamp = frc::Timer::GetFPGATimestamp();
    SetIntakeState(intakeStates::slow);
    SetShooterState(shooterStates::shooterMid);
  }

  if (currIntakeState == intakeStates::slow) {
    rumbleController();
    if (!NoteInShooter()) {
      intakeSpeed = 0.2;
    } else {
      intakeSpeed = 0.0;
    }
  }

  bool atShootAngle = m_drive->atShootingAngle();
  bool driveMode = (m_drive->m_desiredDriveState == m_drive->aimDrive);

  if (currShooterState == shooterStates::shooterOn && driveMode && atShootAngle && ShooterAtSpeed() && m_armSubsystem->ReachedDesiredAngle() && (NoteInShooter() || NoteInIntake())) {
    shooterTimeStamp = frc::Timer::GetFPGATimestamp();
    SetIntakeState(intakeStates::shoot);
  }

  if (currShooterState == shooterStates::shooterOn && currIntakeState != intakeStates::stop && currIntakeState == intakeStates::shoot && (frc::Timer::GetFPGATimestamp() - shooterTimeStamp) > 0.5_s) {
    SetIntakeState(intakeStates::intake);
    SetShooterState(shooterStates::shooterStop);
    m_armSubsystem->SetDesiredAngle(ArmSubsystem::ArmStates::intake);
  }

  if (!NoteInShooter() && currIntakeState == intakeStates::eject && currIntakeState != intakeStates::stop) {
    SetIntakeState(intakeStates::intake);
    SetShooterState(shooterStates::shooterStop);
    m_armSubsystem->SetDesiredAngle(ArmSubsystem::ArmStates::intake);
  }

  runVelocity(shooterSpeed);
  m_intakeMotor.Set(intakeSpeed);
  // intakePID.SetReference(intakeSpeed, rev::CANSparkBase::ControlType::kVelocity);
}

void ShooterSubsystem::SetIntakeState(intakeStates intakeState) {
  currIntakeState = intakeState;
  switch (intakeState) {
    case intakeStates::intake:
      if (shooterIdleChooser.GetSelected() == "On") {
        intakeSpeed = 1;
      } else {
        intakeSpeed = 0.3;
      }
      break;

    case intakeStates::shoot:
      intakeSpeed = 1;
      break;

    case intakeStates::eject:
      intakeSpeed = 1;
      break;

    case intakeStates::stop:
      intakeSpeed = 0;
      break;

    default:
      intakeSpeed = 0.0;
  }
}

void ShooterSubsystem::SetShooterState(shooterStates shooterState) {
  currShooterState = shooterState;
  switch (shooterState) {
    case shooterStates::shooterOn:
      shooterSpeed = frc::SmartDashboard::GetNumber("Shooter Velocity", 6500.0);
      break;

    case shooterStates::shooterStop:
      if (frc::DriverStation::IsAutonomous()) {
        shooterSpeed = frc::SmartDashboard::GetNumber("Shooter Velocity", 6500.0);
      } else {
        shooterSpeed = 2000;
      }
      break;

    case shooterStates::shooterEject:
      shooterSpeed = 1000;
      break;

    case shooterStates::shooterMid:
      shooterSpeed = 4000;
      break;

    default:
      shooterSpeed = 0.0;
  }
}

void ShooterSubsystem::rumbleController() {
  if ((frc::Timer::GetFPGATimestamp() - intakeTimeStamp) < 1_s) {
    m_secondaryController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
    m_visionSubsystem->BlinkLEDs(true);
  } else {
    m_secondaryController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
    m_visionSubsystem->BlinkLEDs(false);
  }
}

bool ShooterSubsystem::NoteInIntake() {
  frc::SmartDashboard::PutBoolean("Intake Beam Break", !intakeBeamBreak.Get());
  return (!intakeBeamBreak.Get());
}

bool ShooterSubsystem::NoteInShooter() {
  frc::SmartDashboard::PutBoolean("Shooter Beam Break", !shooterBeamBreak.Get());
  return (!shooterBeamBreak.Get());
}

bool ShooterSubsystem::NoteInBoth() {
  return (NoteInIntake() && NoteInShooter());
}

bool ShooterSubsystem::ShooterAtSpeed() {
  double shooterThreshold = shooterSpeed - frc::SmartDashboard::GetNumber("Shooter Velocity Threshold", 150.0);
  return (leftShooterEnc.GetVelocity() > shooterThreshold && rightShooterEnc.GetVelocity() > shooterThreshold);
}

void ShooterSubsystem::runVelocity(double rpm) {
  leftShooterPID.SetReference(rpm, rev::CANSparkBase::ControlType::kVelocity);
  rightShooterPID.SetReference(rpm, rev::CANSparkBase::ControlType::kVelocity);
}

void ShooterSubsystem::setPID() {
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

void ShooterSubsystem::Stop() {
  m_leftShooterMotor.StopMotor();
  m_rightShooterMotor.StopMotor();
}