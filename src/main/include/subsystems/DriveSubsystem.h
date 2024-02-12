// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <AHRS.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "SwerveModule.h"
#include <subsystems/VisionSubsystem.h>
#include <frc/controller/PIDController.h>
class DriveSubsystem : public frc2::SubsystemBase
{
public:
    DriveSubsystem(VisionSubsystem*);

    enum DriveStates
    {
        joyStickDrive,
        aimDrive
    };

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    VisionSubsystem* m_visionSubsystem;

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

    void UpdatePoseLimelight(frc::Translation2d pose);

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

    void SetDriveState(DriveStates state);
    DriveStates m_desiredDriveState;
    void getAimAngle();
    double speakerX;

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    void ResetOdometry(frc::Pose2d pose);

    units::meter_t kTrackWidth =
        0.5969_m; // Distance between centers of right and left wheels on robot
    units::meter_t kWheelBase =
        0.5969_m; // Distance between centers of front and back wheels on robot
    units::meter_t kModuleRadius =
        units::meter_t(sqrt(pow(kTrackWidth.value(), 2) + pow(kWheelBase.value(), 2)));

    frc::SwerveDriveKinematics<4> kDriveKinematics{
        frc::Translation2d{kWheelBase / 2, kTrackWidth / 2},
        frc::Translation2d{kWheelBase / 2, -kTrackWidth / 2},
        frc::Translation2d{-kWheelBase / 2, kTrackWidth / 2},
        frc::Translation2d{-kWheelBase / 2, -kTrackWidth / 2}};

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

    frc::PIDController m_aimController{0.1, 0.0, 0.0};

    // Odometry class for tracking robot pose
    // 4 defines the number of modules
    frc::SwerveDrivePoseEstimator<4> m_odometry;
    frc::Translation2d ignorePose{0_m, 0_m};
};
