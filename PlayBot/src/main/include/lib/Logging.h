#pragma once

#include <frc/smartdashboard/SmartDashboard.h>


void log(std::string_view name, double value)
{
    frc::SmartDashboard::PutNumber(name, value);
}

void log(std::string_view name, bool value)
{
    frc::SmartDashboard::PutBoolean(name, value);
}

void log(std::string_view name, std::string_view value)
{
    frc::SmartDashboard::PutString(name, value);
}