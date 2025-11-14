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

#include "lib/hardware/gyro/Navx.h"
#include "lib/hardware/vision/PhotonVision.h"
#include "lib/subsystem/SwerveModule.h"

#include "Constants.h"
#pragma endregion

class Chassis : public frc2::SubsystemBase
{
    public:

        explicit                                 Chassis();
    
        void                                     Periodic() override;
    
        void                                     Drive(const frc::ChassisSpeeds& speeds);
    
        void                                     ZeroHeading();
        
        void                                     ResetWheelAnglesToZero();
        void                                     ResetDriveEncoders();
    
        wpi::array<frc::SwerveModuleState, 4>    GetModuleStates();
        std::array<frc::SwerveModulePosition, 4> GetModulePositions();
    
        void                                     FlipFieldCentric();

        frc::Rotation2d                          GetHeading();
        frc::Pose2d                              GetPose();

        frc::Pose2d                              GetNearestTag();
    
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
        
        std::array<subsystem::SwerveModule, 4> m_swerveModules;

        frc::SwerveDriveKinematics<4> m_kinematics;

        frc::SwerveDrivePoseEstimator<4> m_poseEstimator;   
            
        bool m_isFieldRelative;
    
        hardware::gyro::Navx m_gyro;

        PhotonVision m_vision;
    
        nt::StructArrayPublisher<frc::SwerveModuleState> m_loggedModuleStatePublisher;

        nt::StructPublisher<frc::Pose2d>  m_loggedPosePublisher;

        nt::StructPublisher<frc::ChassisSpeeds> m_loggedDesiredSpeedsPublisher;
};
