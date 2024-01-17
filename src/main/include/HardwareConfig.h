#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/sensors/AbsoluteSensorRange.h>
#include <ctre/phoenix/sensors/SensorInitializationStrategy.h>
#include <ctre/phoenix/sensors/SensorTimeBase.h>
#include "Constants.h"
class HardwareConfig{
    public:
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration TurnMotorConfig;
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration DriveMotorConfig;
    HardwareConfig(){
        /*Swerve Drive Motor Config*/
        
        ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration DriveSupplyLimit{true, 35, 40, 0.1};
        DriveMotorConfig.slot0.kP = 0.1;
        DriveMotorConfig.slot0.kI = 0;
        DriveMotorConfig.slot0.kD = 0.01;
        DriveMotorConfig.slot0.kF = 1023.0 / 21700;
        DriveMotorConfig.supplyCurrLimit = DriveSupplyLimit;
        DriveMotorConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero;
        DriveMotorConfig.voltageCompSaturation = 12;

        /*Swerve Angle Motor Config*/
        ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration AngleSupplyLimit{true, 10, 20, 0.1};
        TurnMotorConfig.slot0.kP = 0.5;
        TurnMotorConfig.slot0.kI = 0;
        TurnMotorConfig.slot0.kD = 15;
        TurnMotorConfig.supplyCurrLimit = AngleSupplyLimit;
        TurnMotorConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero;
        TurnMotorConfig.voltageCompSaturation = 12;

    }
    


};