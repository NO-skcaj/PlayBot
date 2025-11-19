#pragma once
#pragma warning(disable : 4068) // Disable warnings for unknown pragmas

#include <hal/FRCUsageReporting.h>
#include <frc/TimedRobot.h>

#include "lib/Logging.h"
#include "RobotContainer.h"

#include "Constants.h"

class Robot : public frc::TimedRobot
{
    public:

        void RobotInit()     override;
        void RobotPeriodic() override;

    private:

        // Pointer to the autonomous command
        frc2::Command  *m_autonomousCommand = nullptr;

        // Instantiate the Robot container and get a pointer to the class
        RobotContainer *m_robotContainer    = RobotContainer::GetInstance();
};
