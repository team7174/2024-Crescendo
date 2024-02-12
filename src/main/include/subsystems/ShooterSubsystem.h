#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkFlex.h"
#include "Constants.h"
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>
#include <subsystems/ArmSubsystem.h>
#include <frc/controller/SimpleMotorFeedforward.h>

class ShooterSubsystem : public frc2::SubsystemBase
{
public:
  ShooterSubsystem(ArmSubsystem*);
  ArmSubsystem* m_armSubsystem;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void Stop();
  enum intakeStates
  {
    shoot,
    stop,
    eject,
    intake
  };

  enum shooterStates
  {
    shooterOn,
    shooterStop,
    shooterEject
  };

  void SetIntakeState(intakeStates intakeState);
  void SetShooterState(shooterStates shooterState);
  bool NotePresent();
  bool ShooterAtSpeed();
  void runVelocity(double rpm);
  void setPID();

  frc::DigitalInput intakeBeamBreak{1};
  frc::DigitalInput shooterBeamBreak{2};

  intakeStates currIntakeState;
  shooterStates currShooterState;

private:
  rev::CANSparkFlex m_leftShooterMotor;
  rev::CANSparkFlex m_rightShooterMotor;
  rev::CANSparkFlex m_intakeMotor;
  
  rev::SparkRelativeEncoder rightShooterEnc;
  rev::SparkRelativeEncoder leftShooterEnc;

  rev::SparkPIDController leftShooterPID;
  rev::SparkPIDController rightShooterPID;

  double intakeSpeed;
  double shooterSpeed;

  double kP = 0.0;
  double kI = 0.0;
  double kD = 0.0;
};
