#include "subsystems/Chassis.h"

#pragma region Chassis
Chassis::Chassis()
{
    // Set the swerve modules to their forward angles
    ResetWheelAnglesToZero();
}
#pragma endregion

#pragma region Periodic
/// @brief Method called once per scheduler run.
void Chassis::Periodic()
{
    // This method will be called once per scheduler run
    m_loggedModuleStatePublisher.Set(wpi::array<frc::SwerveModuleState, 4>
    {
        m_swerveModules[0].GetState(),
        m_swerveModules[1].GetState(),
        m_swerveModules[2].GetState(),
        m_swerveModules[3].GetState()
    });

    // Log the robot pose
    m_loggedPosePublisher.Set(GetPose());

    // Update odometry
    OdometryPeriodic();
}
#pragma endregion

#pragma region Drive
/// @brief Method to drive the chassis with the specified speeds.
/// @param speeds The desired chassis speeds.
void Chassis::Drive(const frc::ChassisSpeeds& speeds)
{
    // Log the desired speeds
    m_loggedDesiredSpeedsPublisher.Set(speeds);

    // Set the module states
    auto m_desiredStates = m_kinematics.ToSwerveModuleStates(m_isFieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetHeading()) : speeds);

    // Set the desired state for each swerve module
    m_swerveModules[0].SetDesiredState(m_desiredStates[0]);
    m_swerveModules[1].SetDesiredState(m_desiredStates[1]);
    m_swerveModules[2].SetDesiredState(m_desiredStates[2]);
    m_swerveModules[3].SetDesiredState(m_desiredStates[3]);

    // Simulate the gyro in simulation
    if (frc::RobotBase::IsSimulation())
        Gyro::GetInstance()->SimPeriodic(speeds.omega);
}
#pragma endregion

#pragma region ResetWheelAnglesToZero 
/// @brief Method to reset the wheel angles to zero.
void Chassis::ResetWheelAnglesToZero()
{
    // Set the swerve wheel angles to zero
    m_swerveModules[0].SetWheelAngleToForward(constants::swerve::RobotSwerveConfig.frontLeftForwardAngle);
    m_swerveModules[1].SetWheelAngleToForward(constants::swerve::RobotSwerveConfig.frontRightForwardAngle);
    m_swerveModules[2].SetWheelAngleToForward(constants::swerve::RobotSwerveConfig.rearLeftForwardAngle);
    m_swerveModules[3].SetWheelAngleToForward(constants::swerve::RobotSwerveConfig.rearRightForwardAngle);
}
#pragma endregion

#pragma region ResetDriveEncoders
/// @brief Method to reset the drive encoders.
void Chassis::ResetDriveEncoders()
{
    // Reset the swerve motor encoders
    m_swerveModules[0].ResetDriveEncoder();
    m_swerveModules[1].ResetDriveEncoder();
    m_swerveModules[2].ResetDriveEncoder();
    m_swerveModules[3].ResetDriveEncoder();
}
#pragma endregion

#pragma region GetModuleStates
/// @brief Method to get the current swerve module states.
/// @return The current swerve module states.
wpi::array<frc::SwerveModuleState, 4> Chassis::GetModuleStates()
{
    // Return the swerve module states
    std::array<frc::SwerveModuleState, 4> swerveStates =
    {
        m_swerveModules[0].GetState(),
        m_swerveModules[1].GetState(),
        m_swerveModules[2].GetState(),
        m_swerveModules[3].GetState()
    };

    // Adjust for simulation conversions
    if (frc::RobotBase::IsSimulation())
    {
        // Convert the states back to real world units
        for (auto swerveState : swerveStates)
        {
            // Adjust the speed and angle conversions
            swerveState = {swerveState.speed / constants::swerve::RobotSwerveConfig.driveConversion.value(), 
                           swerveState.angle / constants::swerve::RobotSwerveConfig.angleConversion.value()};
        }
    }

    // Return the swerve states
    return swerveStates;
}
#pragma endregion

#pragma region GetModulePositions
/// @brief Method to get the current swerve module positions.
/// @return The current swerve module positions.
std::array<frc::SwerveModulePosition, 4> Chassis::GetModulePositions()
{
    // Return the swerve module states
    std::array<frc::SwerveModulePosition, 4> swervePositions =
    {
        m_swerveModules[0].GetPosition(),
        m_swerveModules[1].GetPosition(),
        m_swerveModules[2].GetPosition(),
        m_swerveModules[3].GetPosition()
    };

    // Adjust for simulation conversions
    if (frc::RobotBase::IsSimulation())
    {
        // Convert the swerve positions back to real world units
        for (auto swervePosition : swervePositions)
        {
            // Adjust the distance and angle conversions
            swervePosition = {swervePosition.distance / constants::swerve::RobotSwerveConfig.driveConversion.value(), 
                              swervePosition.angle    / constants::swerve::RobotSwerveConfig.angleConversion.value()};
        }
    }

    // Return the swerve positions
    return swervePositions;
}
#pragma endregion

#pragma region FlipFieldCentric
/// @brief Method to flip the field centric mode.
void Chassis::FlipFieldCentric()
{
    // Toggle the field relative mode
    m_isFieldRelative = !m_isFieldRelative;
}
#pragma endregion

#pragma region GetHeading
/// @brief Method to get the robot heading.
/// @return The robot heading.
frc::Rotation2d Chassis::GetHeading()
{
    // Return the gyro rotation
    return Gyro::GetInstance()->GetRotation().ToRotation2d();
}
#pragma endregion

#pragma region GetPose
/// @brief Method to get the robot pose.
/// @return The robot pose.
frc::Pose2d Chassis::GetPose()
{
    // Return the estimated robot pose
    return m_poseEstimator.GetEstimatedPosition();
}
#pragma endregion

#pragma region OdometryPeriodic
/// @brief Method called periodically to update odometry.
void Chassis::OdometryPeriodic()
{
    // Update the pose estimator
    m_poseEstimator.Update(GetHeading(), GetModulePositions());

    // This also updates the pose estimator with vision as well as updating photonvisions internal estimators
    m_vision.Periodic();
}
#pragma endregion
