#include "subsystems/PDH.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

PDH::PDH()
{}

void PDH::Periodic()
{
    // frc::SmartDashboard::PutNumber("Battery Voltage", revPDH.GetVoltage());
    // frc::SmartDashboard::PutNumber("Total Current", revPDH.GetTotalCurrent());

    frc::SmartDashboard::PutNumber("FL Steer Current", revPDH.GetCurrent(0));
    frc::SmartDashboard::PutNumber("FL Drive Current", revPDH.GetCurrent(1));

    frc::SmartDashboard::PutNumber("L Climb Current", revPDH.GetCurrent(2));
    frc::SmartDashboard::PutNumber("R Climb Current", revPDH.GetCurrent(17));
    
    frc::SmartDashboard::PutNumber("L Arm Current", revPDH.GetCurrent(3));
    frc::SmartDashboard::PutNumber("R Arm Current", revPDH.GetCurrent(16));

    frc::SmartDashboard::PutNumber("BL Steer Current", revPDH.GetCurrent(8));
    frc::SmartDashboard::PutNumber("BL Drive Current", revPDH.GetCurrent(9));

    frc::SmartDashboard::PutNumber("BR Steer Current", revPDH.GetCurrent(10));
    frc::SmartDashboard::PutNumber("BR Drive Current", revPDH.GetCurrent(11));

    frc::SmartDashboard::PutNumber("FR Steer Current", revPDH.GetCurrent(19));
    frc::SmartDashboard::PutNumber("FR Drive Current", revPDH.GetCurrent(18));

    frc::SmartDashboard::PutNumber("R Shooter Current", revPDH.GetCurrent(12));
    frc::SmartDashboard::PutNumber("L Shooter Current", revPDH.GetCurrent(13));
    frc::SmartDashboard::PutNumber("Intake Current", revPDH.GetCurrent(14));


}