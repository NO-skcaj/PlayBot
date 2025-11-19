#pragma once

#pragma region Includes
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/CommandPtr.h>

#include "subsystems/Volcano.h"
#pragma endregion

#pragma region VolcanoFlywheelOn(Volcano* volcano)
/// @brief Creates a command to turn the volcano flywheel on.
/// @param volcano A pointer to the Volcano subsystem.
/// @return A CommandPtr that turns the flywheel on.
/// TODO: Need to be able to set the flywheel speed.
inline frc2::CommandPtr SetVolcanoFlywheelSpeed(Volcano* volcano, units::turns_per_second_t speed)
{
    // Create and return a FunctionalCommand that turns the flywheel on
    return frc2::InstantCommand{[volcano, speed] () { volcano->SetFlywheel(speed); }, { volcano }}.ToPtr();
}
#pragma endregion

#pragma region VolcanoShootAllBalls(Volcano* volcano)
/// @brief Creates a command to shoot one ball from the volcano.
/// @param volcano A pointer to the Volcano subsystem.
inline frc2::CommandPtr VolcanoShootOneBall(Volcano* volcano)
{
    // Turn on Flywheel and wait until at speed if its not already
    return SetVolcanoFlywheelSpeed(volcano, 1000_tps).Until([volcano]() {
            return volcano->IsFlywheelAtSpeed();
        }
    // Then index and wait until the kick sensor is triggered if its not already
    ).AlongWith(
        frc2::InstantCommand{[volcano]() {
            volcano->SetIndexers(true);
        }, { volcano }}
        .Until([volcano]() {
            return volcano->IsBallDetected();
        })
    // Then kick the ball to the flywheel and wait a second
    ).AndThen(
        [volcano]() {
            volcano->SetKicker(true);
        },
        { volcano }
    ).AndThen(frc2::WaitCommand(1_s).ToPtr()
    // Finally, turn off the kicker and indexers
    ).AndThen(
        [volcano]() {
            volcano->SetKicker(false);
            volcano->SetIndexers(false);
            volcano->SetFlywheel(0_tps);
        },
        { volcano }
    );
}
#pragma endregion

#pragma region VolcanoShootAllBalls(Volcano* volcano)
/// @brief Creates a command to shoot all balls from the volcano.
/// @param volcano A pointer to the Volcano subsystem.
inline frc2::CommandPtr VolcanoShootAllBalls(Volcano* volcano)
{
    // Turn on Flywheel and wait until at speed if its not already
    return SetVolcanoFlywheelSpeed(volcano, 1000_tps).Until([volcano]() {
            return volcano->IsFlywheelAtSpeed();
        }
    // Then activate everything, making all balls go through the system
    ).AndThen(
        frc2::InstantCommand{[volcano]() {
            volcano->SetIndexers(true);
            volcano->SetKicker(true);
            volcano->SetFlywheel(1000_tps);
        }, { volcano }}.ToPtr()
    );
}
#pragma endregion

#pragma region VolcanoStopAll(Volcano* volcano)
/// @brief Creates a command to stop all action in the volcano. HALTS ERUPTION.
/// @param volcano A pointer to the Volcano subsystem.
inline frc2::CommandPtr VolcanoStopAll(Volcano* volcano)
{
    return frc2::InstantCommand{[volcano]() {
            volcano->SetIndexers(false);
            volcano->SetKicker(false);
            volcano->SetFlywheel(0_tps);
        }, { volcano }}.ToPtr();
}
#pragma endregion