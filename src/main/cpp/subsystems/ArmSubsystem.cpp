#include "subsystems/ArmSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ArmSubsystem::ArmSubsystem(DriveSubsystem *passedDriveSubsystem)
    : m_armMotorLeft(StormbreakerConstants::leftArmID),   // Replace with your TalonFX device ID
      m_armMotorRight(StormbreakerConstants::rightArmID), // Replace with your TalonFX device ID
      m_armEncoder(StormbreakerConstants::armEncoderID),
      profiledController(
          StormbreakerConstants::armkP, // Placeholder for proportional gain
          StormbreakerConstants::armkI, // Placeholder for integral gain
          StormbreakerConstants::armkD, // Placeholder for derivative gain
          frc::TrapezoidProfile<units::degrees>::Constraints(ArmSubsystem::kMaxAngularSpeed, ArmSubsystem::kMaxAngularAcceleration))
{
  m_driveSubsystem = passedDriveSubsystem;

  ctre::phoenix6::configs::TalonFXConfiguration ArmMotorConfig{};

  /*Arm Angle Motor Config*/
  auto &slot0ConfigsArm = ArmMotorConfig.Slot0;
  slot0ConfigsArm.kS = 0.25; // Add 0.25 V output to overcome static friction
  slot0ConfigsArm.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
  slot0ConfigsArm.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  slot0ConfigsArm.kP = 1.2;  // An error of 1 rps results in 0.11 V output
  slot0ConfigsArm.kI = 0;    // no output for integrated error
  slot0ConfigsArm.kD = 0;    // No output for change in error

  auto &Armslot0ConfigsArmCurrLimit = ArmMotorConfig.CurrentLimits;
  Armslot0ConfigsArmCurrLimit.StatorCurrentLimit = 35;
  Armslot0ConfigsArmCurrLimit.StatorCurrentLimitEnable = true;

  Armslot0ConfigsArmCurrLimit.SupplyCurrentLimitEnable = true;
  Armslot0ConfigsArmCurrLimit.SupplyCurrentLimit = 35;
  Armslot0ConfigsArmCurrLimit.SupplyCurrentThreshold = 40;
  Armslot0ConfigsArmCurrLimit.SupplyTimeThreshold = 0.1;

  auto &Armslot0ConfigsArmVoltLimit = ArmMotorConfig.Voltage;
  Armslot0ConfigsArmVoltLimit.PeakForwardVoltage = 12;
  Armslot0ConfigsArmVoltLimit.PeakReverseVoltage = -12;

  m_armMotorLeft.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});
  m_armMotorLeft.GetConfigurator().Apply(ArmMotorConfig, 50_ms);
  m_armMotorRight.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});
  m_armMotorRight.GetConfigurator().Apply(ArmMotorConfig, 50_ms);
  profiledController.SetGoal(0.0_deg);
  profiledController.SetTolerance(1_deg);
  m_armMotorRight.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  m_armMotorLeft.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  m_armMotorLeft.SetInverted(true);
  frc::SmartDashboard::PutNumber("Angle Offset", angleOffset);
}

void ArmSubsystem::Periodic()
{
  if (armSwitch.Get() && brakeMode == true)
  {
    brakeModeOff();
  }
  else if(!armSwitch.Get() && brakeMode == false)
  {
    brakeModeOn();
  }
  if (currArmState == ArmStates::autoAngle)
  {
    SetDesiredAngle(ArmStates::autoAngle);
  }
  double speed = std::clamp(profiledController.Calculate(units::degree_t(GetAbsArmAngle())), -1.0, 1.0);
  frc::SmartDashboard::PutNumber("Through Bore Angle", GetAbsArmAngle());
  frc::SmartDashboard::PutNumber("Desired Angle", profiledController.GetGoal().position());
  frc::SmartDashboard::PutNumber("Calculated Angle", CalculateAngle());
  m_armMotorLeft.Set(speed);
  m_armMotorRight.Set(speed);
}

void ArmSubsystem::brakeModeOff()
{
  brakeMode = false;
  m_armMotorRight.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
  m_armMotorLeft.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
}

void ArmSubsystem::brakeModeOn()
{
  brakeMode = true;
  m_armMotorRight.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  m_armMotorLeft.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

double ArmSubsystem::GetAbsArmAngle()
{
  return abs((m_armEncoder.GetAbsolutePosition() - StormbreakerConstants::armEncoderOffset) * 360);
}

void ArmSubsystem::SetDesiredAngle(ArmStates DesiredArmState)
{
  currArmState = DesiredArmState;
  // Update the setpoint of the PID controller
  if (DesiredArmState == ArmStates::intake)
  {
    profiledController.SetGoal(0_deg);
  }
  else if (DesiredArmState == ArmStates::upright)
  {
    profiledController.SetGoal(90_deg);
  }
  else if (DesiredArmState == ArmStates::autoAngle)
  {
    profiledController.SetGoal(units::degree_t(CalculateAngle()));
    //profiledController.SetGoal(units::degree_t(0));
  }
  profiledController.SetGoal(units::degree_t(std::clamp(profiledController.GetGoal().position(), 0.0, 100.0)));
}

double ArmSubsystem::CalculateAngle()
{
  double distanceToSpeaker = m_driveSubsystem->getShootingValues().first - 0.2032;
  double speakerHeight = m_driveSubsystem->GetSpeakerCenter().Z().value() - 0.2032;

  double shootingAngle = atan(distanceToSpeaker / speakerHeight) + asin((sin(65 * M_PI / 180) * StormbreakerConstants::armLength) / (sqrt(pow(distanceToSpeaker, 2) + pow(speakerHeight, 2)))) - (25 * M_PI / 180);

  shootingAngle = fmod(shootingAngle * (180 / M_PI), 360.0);
  shootingAngle = shootingAngle - frc::SmartDashboard::GetNumber("Angle Offset", angleOffset);

  return shootingAngle;
}

bool ArmSubsystem::ReachedDesiredAngle()
{
  frc::SmartDashboard::PutBoolean("At Desired Angle", profiledController.AtGoal());
  return profiledController.AtGoal();
}

void ArmSubsystem::Stop()
{
  m_armMotorLeft.StopMotor();
  m_armMotorRight.StopMotor();
}
