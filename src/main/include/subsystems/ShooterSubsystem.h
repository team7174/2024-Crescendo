#pragma once

#include <frc/DigitalInput.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/timer.h>
#include <frc2/command/SubsystemBase.h>
#include <subsystems/ArmSubsystem.h>

#include "Constants.h"
#include "rev/CANSparkFlex.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem(ArmSubsystem *, DriveSubsystem *, VisionSubsystem *, frc::XboxController *, frc::XboxController *);
  ArmSubsystem *m_armSubsystem;
  DriveSubsystem *m_drive;
  VisionSubsystem *m_visionSubsystem;
  frc::XboxController *m_secondaryController;
  frc::XboxController *m_driveController;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void Stop();
  enum intakeStates {
    shoot,
    slow,
    stop,
    eject,
    intake,
    amp,
    spit
  };

  enum shooterStates {
    shooterOn,
    shooterStop,
    shooterMid,
    shooterEject
  };

  void SetIntakeState(intakeStates intakeState);
  void SetShooterState(shooterStates shooterState);
  bool NoteInIntake();
  bool NoteInShooter();
  bool NoteInBoth();
  bool ShooterAtSpeed();
  void runVelocity(double rpm);
  void setPID();
  void rumbleController();

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

  rev::SparkRelativeEncoder intakeEnc;
  rev::SparkPIDController intakePID;

  double intakeSpeed;
  double shooterSpeed;

  units::time::second_t shooterTimeStamp;
  units::time::second_t intakeTimeStamp;

  frc::DigitalInput intakeBeamBreak{ShooterConstants::intakeBeamBreakID};
  frc::DigitalInput shooterBeamBreak{ShooterConstants::shooterBeamBreakID};

  frc::SendableChooser<std::string> shooterIdleChooser;
};