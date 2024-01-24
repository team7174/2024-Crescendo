#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include "Constants.h"

class HardwareConfig
{
public:
    ctre::phoenix6::configs::Slot0Configs TurnMotorConfig{};
    ctre::phoenix6::configs::Slot0Configs DriveMotorConfig{};
    ctre::phoenix6::configs::CurrentLimitsConfigs DriveCurrLimit{};
    ctre::phoenix6::configs::CurrentLimitsConfigs TurnCurrLimit{};
    ctre::phoenix6::configs::VoltageConfigs DriveVoltageLimit{};
    ctre::phoenix6::configs::VoltageConfigs TurnVoltageLimit{};
    HardwareConfig()
    {
        /*Swerve Drive Motor Config*/
        DriveMotorConfig.kP = 0.1;
        DriveMotorConfig.kI = 0;
        DriveMotorConfig.kD = 0.01;
        DriveMotorConfig.kV = 1023.0 / 21700;

        DriveCurrLimit.StatorCurrentLimit = 10;
        DriveCurrLimit.StatorCurrentLimitEnable = true;

        DriveCurrLimit.SupplyCurrentLimitEnable = true;
        DriveCurrLimit.SupplyCurrentLimit = 35;
        DriveCurrLimit.SupplyCurrentThreshold = 40;
        DriveCurrLimit.SupplyTimeThreshold = 0.1;

        DriveVoltageLimit.PeakForwardVoltage = 12;
        DriveVoltageLimit.PeakReverseVoltage = -12;

        // DriveMotorConfig.initializationStrategy = ctre::phoenix6::configs::BootToZero;

        /*Swerve Angle Motor Config*/
        TurnMotorConfig.kP = 0.5;
        TurnMotorConfig.kI = 0;
        TurnMotorConfig.kD = 15;

        TurnCurrLimit.StatorCurrentLimit = 10;
        TurnCurrLimit.StatorCurrentLimitEnable = true;

        TurnCurrLimit.SupplyCurrentLimitEnable = true;
        TurnCurrLimit.SupplyCurrentLimit = 10;
        TurnCurrLimit.SupplyCurrentThreshold = 20;
        TurnCurrLimit.SupplyTimeThreshold = 0.1;

        TurnVoltageLimit.PeakForwardVoltage = 12;
        TurnVoltageLimit.PeakReverseVoltage = -12;

        // TurnMotorConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero;
    }
};