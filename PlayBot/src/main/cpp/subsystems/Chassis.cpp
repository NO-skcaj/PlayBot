#include "subsystems/Chassis.h"

#pragma region Chassis
Chassis::Chassis()
    : m_swerveModules
      {
          subsystem::SwerveModule{constants::swerve::frontLeftDriveCANid,  constants::swerve::frontLeftTurnCANid,  constants::swerve::frontLeftEncoderCANid,  constants::swerve::driveMotorConfig, constants::swerve::turnMotorConfig, constants::swerve::driveConversion, constants::swerve::angleConversion},
          subsystem::SwerveModule{constants::swerve::frontRightDriveCANid, constants::swerve::frontRightTurnCANid, constants::swerve::frontRightEncoderCANid, constants::swerve::driveMotorConfig, constants::swerve::turnMotorConfig, constants::swerve::driveConversion, constants::swerve::angleConversion},
          subsystem::SwerveModule{constants::swerve::backLeftDriveCANid,   constants::swerve::backLeftTurnCANid,   constants::swerve::backLeftEncoderCANid,   constants::swerve::driveMotorConfig, constants::swerve::turnMotorConfig, constants::swerve::driveConversion, constants::swerve::angleConversion},
          subsystem::SwerveModule{constants::swerve::backRightDriveCANid,  constants::swerve::backRightTurnCANid,  constants::swerve::backRightEncoderCANid,  constants::swerve::driveMotorConfig, constants::swerve::turnMotorConfig, constants::swerve::driveConversion, constants::swerve::angleConversion}
      },
      m_kinematics
      {
          frc::Translation2d{+constants::swerve::wheelBase / 2, +constants::swerve::trackWidth / 2}, // Front Left
          frc::Translation2d{+constants::swerve::wheelBase / 2, -constants::swerve::trackWidth / 2}, // Front Right
          frc::Translation2d{-constants::swerve::wheelBase / 2, +constants::swerve::trackWidth / 2}, // Back Left
          frc::Translation2d{-constants::swerve::wheelBase / 2, -constants::swerve::trackWidth / 2}  // Back Right
      },
      m_poseEstimator
      {
          m_kinematics,                                 // Kinematics object
          frc::Rotation2d(),                            // Initial gyro angle
          std::array<frc::SwerveModulePosition, 4>{},   // Initial module positions
          frc::Pose2d()                                 // Initial pose
      },
      m_isFieldRelative{true},
      m_vision
      {
        constants::vision::CameraName,
        constants::vision::RobotToCam,
        constants::vision::TagLayout,
        constants::vision::SingleTagStdDevs,
        constants::vision::MultiTagStdDevs,
        [this] (frc::Pose2d pose, units::second_t timestamp, Eigen::Matrix<double, 3, 1> stddevs)
        {
           m_poseEstimator.AddVisionMeasurement(pose, timestamp, {stddevs[0], stddevs[1], stddevs[2]});
        }
      },
      m_loggedModuleStatePublisher{
            nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>("/Data/SwerveStates").Publish()},
       m_loggedPosePublisher{
         nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/Data/CurrentPose").Publish()},
      m_loggedDesiredSpeedsPublisher{
        nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::ChassisSpeeds>("/Data/DesiredSpeeds").Publish()}
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
       m_gyro.SimPeriodic(speeds.omega);
}
#pragma endregion

#pragma region ZeroHeading
/// @brief Method to zero the robot heading.
void Chassis::ZeroHeading()
{
    // Zero the gyro heading
    m_gyro.ResetYaw();
}
#pragma endregion

#pragma region ResetWheelAnglesToZero 
/// @brief Method to reset the wheel angles to zero.
void Chassis::ResetWheelAnglesToZero()
{
    // Set the swerve wheel angles to zero
    m_swerveModules[0].SetWheelAngleToForward(constants::swerve::frontLeftForwardAngle);
    m_swerveModules[1].SetWheelAngleToForward(constants::swerve::frontRightForwardAngle);
    m_swerveModules[2].SetWheelAngleToForward(constants::swerve::rearLeftForwardAngle);
    m_swerveModules[3].SetWheelAngleToForward(constants::swerve::rearRightForwardAngle);
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
    return m_gyro.GetRotation().ToRotation2d();
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

#pragma region GetNearestTag
/// @brief Method to get the nearest AprilTag pose.
/// @return The nearest AprilTag pose.
frc::Pose2d Chassis::GetNearestTag()
{
    return GetPose().Nearest(constants::vision::AprilTagLocations::Pose2dTagsSpan);
}
#pragma endregion