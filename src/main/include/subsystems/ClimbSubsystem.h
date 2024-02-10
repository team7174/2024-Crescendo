#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DutyCycleEncoder.h>
#include <ctre/Phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include "Constants.h"
#include <frc/XboxController.h>

class ClimbSubsystem : public frc2::SubsystemBase
{
public:
    ClimbSubsystem();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    enum ClimbStates
    {
        retract,
        extend,
        hold
    };
    void SetClimbState(ClimbStates DesiredClimbState);
    void Stop();

private:
    ctre::phoenix6::hardware::TalonFX m_climbMotorLeft;
    ctre::phoenix6::hardware::TalonFX m_climbMotorRight;
    frc::PIDController m_climbPIDController;
};