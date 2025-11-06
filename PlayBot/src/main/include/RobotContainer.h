#pragma once

#pragma region Includes
#include <utility>
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
};
