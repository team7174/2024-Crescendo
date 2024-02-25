#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/PowerDistribution.h>

class PDH : public frc2::SubsystemBase
{
public:
    PDH();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    void Stop();
    frc::PowerDistribution revPDH{1, frc::PowerDistribution::ModuleType::kRev};

private:

};
