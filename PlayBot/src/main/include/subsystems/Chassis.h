#pragma once

#pragma region Includes
#include <wpi/array.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include <frc2/command/SubsystemBase.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include "lib/subsystem/SwerveModule.h"

#include "subsystems/Vision.h"
#include "subsystems/Gyro.h"

#include "Constants.h"
#pragma endregion

class Chassis : public frc2::SubsystemBase
{
    public:

        explicit                                 Chassis();
    
        void                                     Periodic() override;
    
        void                                     Drive(const frc::ChassisSpeeds& speeds);
    
        void                                     ResetWheelAnglesToZero();
        void                                     ResetDriveEncoders();
    
        wpi::array<frc::SwerveModuleState, 4>    GetModuleStates();
        std::array<frc::SwerveModulePosition, 4> GetModulePositions();
    
        void                                     FlipFieldCentric();

        frc::Rotation2d                          GetHeading();
        frc::Pose2d                              GetPose();
    
    private:
    
        void OdometryPeriodic();
        
        // Swerve module order for kinematics calculations
        //
        //         Front          Translation2d Coordinates
        //   FL +----------+ FR              ^ X
        //      | 0      1 |                 |
        //      |          |            Y    |
        //      |          |          <------+-------
        //      | 2      3 |                 |
        //   RL +----------+ RR              |
        std::array<subsystem::SwerveModule, 4> m_swerveModules
        {
            subsystem::SwerveModule{constants::swerve::RobotSwerveConfig.frontLeftDriveCANid,  constants::swerve::RobotSwerveConfig.frontLeftTurnCANid,  constants::swerve::RobotSwerveConfig.frontLeftEncoderCANid,  constants::swerve::RobotSwerveConfig.driveMotorConfig, constants::swerve::RobotSwerveConfig.turnMotorConfig, constants::swerve::RobotSwerveConfig.driveConversion, constants::swerve::RobotSwerveConfig.angleConversion},
            subsystem::SwerveModule{constants::swerve::RobotSwerveConfig.frontRightDriveCANid, constants::swerve::RobotSwerveConfig.frontRightTurnCANid, constants::swerve::RobotSwerveConfig.frontRightEncoderCANid, constants::swerve::RobotSwerveConfig.driveMotorConfig, constants::swerve::RobotSwerveConfig.turnMotorConfig, constants::swerve::RobotSwerveConfig.driveConversion, constants::swerve::RobotSwerveConfig.angleConversion},
            subsystem::SwerveModule{constants::swerve::RobotSwerveConfig.backLeftDriveCANid,   constants::swerve::RobotSwerveConfig.backLeftTurnCANid,   constants::swerve::RobotSwerveConfig.backLeftEncoderCANid,   constants::swerve::RobotSwerveConfig.driveMotorConfig, constants::swerve::RobotSwerveConfig.turnMotorConfig, constants::swerve::RobotSwerveConfig.driveConversion, constants::swerve::RobotSwerveConfig.angleConversion},
            subsystem::SwerveModule{constants::swerve::RobotSwerveConfig.backRightDriveCANid,  constants::swerve::RobotSwerveConfig.backRightTurnCANid,  constants::swerve::RobotSwerveConfig.backRightEncoderCANid,  constants::swerve::RobotSwerveConfig.driveMotorConfig, constants::swerve::RobotSwerveConfig.turnMotorConfig, constants::swerve::RobotSwerveConfig.driveConversion, constants::swerve::RobotSwerveConfig.angleConversion}
        };

        frc::SwerveDriveKinematics<4> m_kinematics
        {
            frc::Translation2d{+constants::swerve::RobotSwerveConfig.wheelBase / 2, +constants::swerve::RobotSwerveConfig.trackWidth / 2}, // Front Left
            frc::Translation2d{+constants::swerve::RobotSwerveConfig.wheelBase / 2, -constants::swerve::RobotSwerveConfig.trackWidth / 2}, // Front Right
            frc::Translation2d{-constants::swerve::RobotSwerveConfig.wheelBase / 2, +constants::swerve::RobotSwerveConfig.trackWidth / 2}, // Back Left
            frc::Translation2d{-constants::swerve::RobotSwerveConfig.wheelBase / 2, -constants::swerve::RobotSwerveConfig.trackWidth / 2}  // Back Right
        };

        frc::SwerveDrivePoseEstimator<4> m_poseEstimator
        {
            m_kinematics,                                 // Kinematics object
            frc::Rotation2d(),                            // Initial gyro angle
            std::array<frc::SwerveModulePosition, 4>{},   // Initial module positions
            frc::Pose2d()                                 // Initial pose
        };   
            
        bool m_isFieldRelative = true;
    
        PhotonVision m_vision
        {
            [this] (frc::Pose2d pose, units::second_t timestamp, Eigen::Matrix<double, 3, 1> stddevs)
            {
               m_poseEstimator.AddVisionMeasurement(pose, timestamp, {stddevs[0], stddevs[1], stddevs[2]});
            }
        };
    
        nt::StructArrayPublisher<frc::SwerveModuleState> m_loggedModuleStatePublisher
        {
            // Publish the module states to NetworkTables
            nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>("/Data/SwerveStates").Publish()
        };

        nt::StructPublisher<frc::Pose2d>  m_loggedPosePublisher
        {
            // Publish the robot pose to NetworkTables
            nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/Data/CurrentPose").Publish()
        };

        nt::StructPublisher<frc::ChassisSpeeds> m_loggedDesiredSpeedsPublisher
        {
            // Publish the desired chassis speeds to NetworkTables
            nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::ChassisSpeeds>("/Data/DesiredSpeeds").Publish()
        };
};
