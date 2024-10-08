// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <AHRS.h>
#include <frc/controller/PIDController.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>
#include <subsystems/VisionSubsystem.h>

#include "Constants.h"
#include "SwerveModule.h"
class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem(VisionSubsystem *);

  enum DriveStates {
    joyStickDrive,
    aimDrive,
    noteDrive
  };

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  VisionSubsystem *m_visionSubsystem;

  // Subsystem methods go here.

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  wpi::array<frc::SwerveModuleState, 4> GetModuleStates();

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  units::degree_t GetHeading();

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  bool atShootingAngle();

  frc::Translation3d GetSpeakerCenter();

  void SetDriveState(DriveStates state);
  DriveStates m_desiredDriveState;
  std::pair<double, double> getShootingValues();

  frc2::CommandPtr GeneratedPath(frc::Pose2d targetPose);

  frc::Pose2d FlipPose(frc::Pose2d);

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  units::meter_t kTrackWidth =
      0.5969_m;  // Distance between centers of right and left wheels on robot
  units::meter_t kChassisLength =
      0.5969_m;  // Distance between centers of right and left wheels on robot
  units::meter_t kWheelBase =
      0.4953_m;  // Distance between centers of front and back wheels on robot
  units::meter_t kModuleRadius =
      units::meter_t(sqrt(pow(kTrackWidth.value(), 2) + pow(kWheelBase.value(), 2)) / 2);

  frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d{(kChassisLength / 2) - DriveConstants::frontWheelOffset, kTrackWidth / 2},
      frc::Translation2d{(kChassisLength / 2) - DriveConstants::frontWheelOffset, -kTrackWidth / 2},
      frc::Translation2d{-kChassisLength / 2, kTrackWidth / 2},
      frc::Translation2d{-kChassisLength / 2, -kTrackWidth / 2}};

  frc::Translation3d topRightSpeaker = frc::Translation3d(0.458597_m, 6.065901_m, 2.1105114_m);
  frc::Translation3d topLeftSpeaker = frc::Translation3d(0.458597_m, 5.023231_m, 2.1105114_m);

  frc::Translation3d bottomRightSpeaker = frc::Translation3d(0_m, 6.065901_m, 1.9894296_m);
  frc::Translation3d bottomLeftSpeaker = frc::Translation3d(0_m, 5.023231_m, 1.9894296_m);

  frc::Pose2d stageRight = frc::Pose2d(4.4_m, 3.2_m, frc::Rotation2d(-120_deg));
  frc::Pose2d stageLeft = frc::Pose2d(4.4_m, 5_m, frc::Rotation2d(120_deg));
  frc::Pose2d stageFront = frc::Pose2d(5.85_m, 4.1_m, frc::Rotation2d(0_deg));

  bool allianceColorBlue;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::Field2d m_field;

  SwerveModule m_frontLeft;
  SwerveModule m_rearLeft;
  SwerveModule m_frontRight;
  SwerveModule m_rearRight;

  // The gyro sensor
  AHRS m_gyro{frc::SPI::kMXP};
  frc::ProfiledPIDController<units::radian> profiledAimController;
  frc::ProfiledPIDController<units::radian> profiledNoteController;

  // Odometry class for tracking robot pose
  // 4 defines the number of modules
  frc::SwerveDrivePoseEstimator<4> m_odometry;
  frc::Translation2d ignorePose{0_m, 0_m};

  double aimThreshold = 3;
};