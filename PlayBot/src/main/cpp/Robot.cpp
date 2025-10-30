#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/print.h>

Robot::Robot()
{
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);

    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Robot::RobotPeriodic() 
{

}

void Robot::AutonomousInit()
{
    m_autoSelected = m_chooser.GetSelected();

    wpi::print("Auto selected: {}\n", m_autoSelected);

    if (m_autoSelected == kAutoNameCustom)
    {
        // Custom Auto goes here
    }
    else
    {
        // Default Auto goes here
    }
}

void Robot::AutonomousPeriodic()
{
    if (m_autoSelected == kAutoNameCustom)
    {
        // Custom Auto goes here
    }
    else
    {
        // Default Auto goes here
    }
}

void Robot::TeleopInit() 
{

}

void Robot::TeleopPeriodic() 
{

}

void Robot::DisabledInit() 
{

}

void Robot::DisabledPeriodic() 
{

}

void Robot::TestInit() 
{

}

void Robot::TestPeriodic() 
{

}

void Robot::SimulationInit() 
{

}

void Robot::SimulationPeriodic()
{

}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
