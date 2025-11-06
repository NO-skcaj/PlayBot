#pragma once

#include <frc/TimedRobot.h>
#include <hal/FRCUsageReporting.h>

#include "lib/Logging.h"
#include "Controller.h"
#include "RobotContainer.h"

class Robot : public frc::TimedRobot
{
    public:

        Robot();

        void RobotPeriodic() override;

    private:

        // Instantiate the Robot container and get a pointer to the class
        RobotContainer *m_robotContainer = RobotContainer::GetInstance();
};
