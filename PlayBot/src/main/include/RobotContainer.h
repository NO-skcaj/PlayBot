#pragma once

#pragma region Includes
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include <frc/filter/SlewRateLimiter.h>

#include "commands/Chassis.h"
#include "commands/Volcano.h"

#include "Constants.h"
#pragma endregion

/// @brief Class to instantiate the robot subsystems and commands along with the operator controls
class RobotContainer
{
    public:

        // Method that returns a pointer to the singleton instance of the RobotContainer class
        static RobotContainer *GetInstance();

    private:

        // Private class constructor to configure the robot and SmartDashboard configuration
        RobotContainer();

        // Singleton reference to the class (returned by the GetInstance Method)
        static RobotContainer *m_robotContainer;

        std::function<frc::ChassisSpeeds()> GetChassisSpeeds();

        double                              GetExponentialValue(double joystickValue, double exponent);
            
        frc::XboxController                 m_driveController{constants::controller::DrivePort};

        frc::SlewRateLimiter<units::scalar> m_flywheelLimiter{1.0 / 0.5_s};  // Full throttle change in 0.5 seconds;

        Chassis                            *m_chassis;
        Volcano                            *m_volcano;
};
