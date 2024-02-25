#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include "Constants.h"

class HardwareConfig
{
public:
    ctre::phoenix6::configs::TalonFXConfiguration TurnMotorConfig{};
    ctre::phoenix6::configs::TalonFXConfiguration DriveMotorConfig{};
    HardwareConfig()
    {
        /*Swerve Drive Motor Config*/
        auto &slot0Configs = DriveMotorConfig.Slot0;
        slot0Configs.kS = 0.5; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.25; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.025; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.2;  // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0;    // no output for integrated error
        slot0Configs.kD = 0;    // no output for error derivative
                                //  DriveMotorConfig.kS = 0.05; // static friction

        // // set Motion Magic Velocity settings
        auto &motionMagicConfigs = DriveMotorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 800; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000;        // Target jerk of 4000 rps/s/s (0.1 seconds)

        auto &DriveCurrLimit = DriveMotorConfig.CurrentLimits;
        DriveCurrLimit.StatorCurrentLimit = 30;
        DriveCurrLimit.StatorCurrentLimitEnable = true;

        DriveCurrLimit.SupplyCurrentLimitEnable = true;
        DriveCurrLimit.SupplyCurrentLimit = 30;
        DriveCurrLimit.SupplyCurrentThreshold = 35;
        DriveCurrLimit.SupplyTimeThreshold = 0.1;

        auto &DriveVoltLimit = DriveMotorConfig.Voltage;
        DriveVoltLimit.PeakForwardVoltage = 12;
        DriveVoltLimit.PeakReverseVoltage = -12;

        // DriveMotorConfig.initializationStrategy = ctre::phoenix6::configs::BootToZero;

        /*Swerve Angle Motor Config*/
        auto &slot0ConfigsTurn = TurnMotorConfig.Slot0;
        slot0ConfigsTurn.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0ConfigsTurn.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
        slot0ConfigsTurn.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0ConfigsTurn.kP = 1.2;  // An error of 1 rps results in 0.11 V output
        slot0ConfigsTurn.kI = 0;    // no output for integrated error
        slot0ConfigsTurn.kD = 0;    // no output for error derivative
        
        auto &TurnCurrLimit = TurnMotorConfig.CurrentLimits;
        TurnCurrLimit.StatorCurrentLimit = 15;
        TurnCurrLimit.StatorCurrentLimitEnable = true;

        TurnCurrLimit.SupplyCurrentLimitEnable = true;
        TurnCurrLimit.SupplyCurrentLimit = 15;
        TurnCurrLimit.SupplyCurrentThreshold = 20;
        TurnCurrLimit.SupplyTimeThreshold = 0.1;

        auto &TurnVoltLimit = TurnMotorConfig.Voltage;
        TurnVoltLimit.PeakForwardVoltage = 12;
        TurnVoltLimit.PeakReverseVoltage = -12;
        // TurnMotorConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero;
    }
};