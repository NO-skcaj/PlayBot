#pragma once

#include <frc/TimedRobot.h>
#include <hal/FRCUsageReporting.h>

#include "lib/Logging.h"
#include "Controller.h"

class Robot : public frc::TimedRobot 
{

  public:

    Robot();
    
    void RobotPeriodic() override;
    
};
