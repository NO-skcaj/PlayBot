#pragma once

#include <functional>

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/WaitCommand.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>

#include "subsystem/Swerve.h"
#include "subsystem/Volcano.h"
#include "subsystem/Gyro.h"

#include "Constants.h"

// TODO: Separate Drive and Volcano commands into their own files

#pragma region Drive Commands
#pragma region ChassisZeroHeading
/// @brief Creates a command to zero the heading of the gyro.
/// @param gyro A pointer to the Gyro subsystem.
/// @return A CommandPtr that resets the gyro yaw to zero.
inline frc2::CommandPtr ChassisZeroHeading(Gyro* gyro)
{
    // Create and return a FunctionalCommand that resets the gyro yaw
    return frc2::FunctionalCommand{
        []     ()                { },                   // Initialization function (runs once when the command starts)  
        []     ()                { },                   // Execution function (runs repeatedly while the command is active)
        [gyro] (bool interupted) { gyro->ResetYaw(); }, // End function (runs once when the command ends, either interrupted or completed)
        []     ()                { return true; },      // IsFinished function (determines when the command should end)
        {}                                              // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion

#pragma region ChassisDrive
 /// @brief Creates a command to drive the chassis using the provided Swerve and chassis speeds supplier.
///  @param swerve A pointer to the Swerve subsystem.
///  @param chassisSpeedsSupplier A function that supplies the desired chassis speeds.
///  @return A CommandPtr that executes the chassis drive functionality.
inline frc2::CommandPtr ChassisDrive(Swerve* swerve, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier)
{
    // Create and return a FunctionalCommand that drives the chassis
    return frc2::FunctionalCommand{
        []                              ()                { },                                         // Initialization function (runs once when the command starts)
        [swerve, chassisSpeedsSupplier] ()                { swerve->Drive(chassisSpeedsSupplier()); }, // Execution function (runs repeatedly while the command is active)
        []                              (bool interupted) { },                                         // End function (runs once when the command ends, either interrupted or completed) 
        []                              ()                { return false; },                           // IsFinished function (determines when the command should end)  
        { swerve }                                                                                     // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion

#pragma region FlipFieldCentricity
/// @brief Creates a command to flip the field centricity of the Swerve.
/// @param swerve A pointer to the Swerve subsystem. 
/// @return A CommandPtr that flips the field centricity.
inline frc2::CommandPtr FlipFieldCentricity(Swerve* swerve)
{
    // Create and return a FunctionalCommand that flips the field centricity
    return frc2::FunctionalCommand{
        []       ()                { },                             // Initialization function (runs once when the command starts)
        [swerve] ()                { swerve->FlipFieldCentric(); }, // Execution function (runs repeatedly while the command is active)
        []       (bool interupted) { },                             // End function (runs once when the command ends, either interrupted or completed)
        []       ()                { return true; },                // IsFinished function (determines when the command should end)
        { swerve }                                                  // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion

#pragma region ChassisDrivePose
/// @brief Creates a command to drive the chassis to a specified pose using the Swerve.
/// @param swerve A pointer to the Swerve subsystem.
/// @param CommandName The name of the command or path to follow.
/// @return A CommandPtr that drives the chassis to the specified pose.
inline frc2::CommandPtr ChassisDrivePose(Swerve* swerve, std::string CommandName)
{
    //return AutoBuilder::followPath(PathPlannerPath::fromPathFile(CommandName));

    // Note: Temporary fix for pathplanner not working correctly when called immediately after another command
    return frc2::WaitCommand(0.1_s).ToPtr(); 
}
#pragma endregion

#pragma region ChassisDrivePose (Pose2d)
/// @brief Creates a command to drive the chassis to a specified pose using the Swerve.
/// @param swerve A pointer to the Swerve subsystem.
/// @param targetPose The target pose to drive to. End goal state relative to the origin, blue alliance side.
/// @return A CommandPtr that drives the chassis to the specified pose.
inline frc2::CommandPtr ChassisDrivePose(Swerve* swerve, frc::Pose2d targetPose)
{
    // return AutoBuilder::pathfindToPose(targetPose, constants::PathPlanner::Constraints);

    // Note: Temporary fix for pathplanner not working correctly when called immediately after another command
    return frc2::WaitCommand(0.1_s).ToPtr(); 
}
#pragma endregion

#pragma endregion AlignToNearestTag
// // This command will align the robot to the nearest AprilTag
// // It will use the AprilTag's pose to determine the target position and rotation
// // The robot will drive towards the target position and rotate to face the target rotation
// inline frc2::CommandPtr AlignToNearestTag(Swerve* swerve, frc::Transform2d targetOffset)
// {
//     // This doesn't need to be a variable. When I wrote this, I just really liked using lambdas. Now, it kinda needs to be because its not in its own class
//     std::function<frc::Pose2d(frc::Pose2d, frc::Transform2d)> getTargetWithOffset = 
//         [] (frc::Pose2d targetPosition, frc::Transform2d targetOffset)
//         {
//             // Rotate offset
//             return frc::Pose2d{
//                 targetPosition.X() +                  targetOffset.Translation().X() * std::cos(targetPosition.Rotation().Radians().value()) - targetOffset.Translation().Y() * std::sin(targetPosition.Rotation().Radians().value()),
//                 targetPosition.Y() +                  targetOffset.Translation().X() * std::sin(targetPosition.Rotation().Radians().value()) + targetOffset.Translation().Y() * std::cos(targetPosition.Rotation().Radians().value()),
//                 targetPosition.Rotation().Degrees() + targetOffset.Rotation().Degrees()};
//         };

//     return ChassisDrivePose(getTargetWithOffset(PhotonVision::GetInstance()->GetNearestTag(), targetOffset));
// }
#pragma endregion
#pragma endregion

#pragma region Valcano Commands
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
#pragma endregion
