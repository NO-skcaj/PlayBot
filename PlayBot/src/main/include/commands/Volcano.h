#pragma once

#pragma region Includes
#include <functional>

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/WaitCommand.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>

#include "subsystem/Volcano.h"

#include "Constants.h"
#pragma endregion

#pragma region VolcanoFlywheelOn
/// @brief Creates a command to turn the volcano flywheel on.
/// @param volcano A pointer to the Volcano subsystem.
/// @return A CommandPtr that turns the flywheel on.
/// TODO: Need to be able to set the flywheel speed.
inline frc2::CommandPtr VolcanoFlywheelOn(Volcano* volcano)
{
    // Create and return a FunctionalCommand that turns the flywheel on
    return frc2::FunctionalCommand{
        []        ()                { },                             // Initialization function (runs once when the command starts)
        [volcano] ()                { volcano->SetFlywheel(true); }, // Execution function (runs repeatedly while the command is active)
        []        (bool interupted) { },                             // End function (runs once when the command ends, either interrupted or completed)
        []        ()                { return true; },                // IsFinished function (determines when the command should end)
        { volcano }                                                  // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion

#pragma region VolcanoFlywheelOff
/// @brief Creates a command to turn the volcano flywheel off.
/// @param volcano A pointer to the Volcano subsystem.
/// @return A CommandPtr that turns the flywheel off.
inline frc2::CommandPtr VolcanoFlywheelOff(Volcano* volcano)
{
    // Create and return a FunctionalCommand that turns the flywheel off
    return frc2::FunctionalCommand{
        []        ()                { },                              // Initialization function (runs once when the command starts)
        [volcano] ()                { volcano->SetFlywheel(false); }, // Execution function (runs repeatedly while the command is active)
        []        (bool interupted) { },                              // End function (runs once when the command ends, either interrupted or completed)
        []        ()                { return true; },                 // IsFinished function (determines when the command should end)
        { volcano }                                                   // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion

#pragma region VolcanoIndexerControl
/// @brief Creates a command to control the volcano indexer using a provided getter function.
/// @param volcano A pointer to the Volcano subsystem.
/// @param arbitaryGetter A function that returns the desired indexer speed.
/// @return A CommandPtr that controls the indexer speed.
inline frc2::CommandPtr VolcanoIndexerControl(Volcano* volcano, std::function<double()> arbitaryGetter)
{
    // Create and return a FunctionalCommand that controls the indexer speed
    return frc2::FunctionalCommand{
        []                        ()                { },                                        // Initialization function (runs once when the command starts)
        [volcano, arbitaryGetter] ()                { volcano->SetIndexer(arbitaryGetter()); }, // Execution function (runs repeatedly while the command is active)
        []                        (bool interupted) { },                                        // End function (runs once when the command ends, either interrupted or completed)
        []                        ()                { return false; },                          // IsFinished function (determines when the command should end)
        { volcano }                                                                             // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion
