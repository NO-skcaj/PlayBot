#pragma once

#pragma region Includes
#include <frc/smartdashboard/SmartDashboard.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

// Types
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#pragma endregion

inline void Log(std::string_view name, double value)
{
    frc::SmartDashboard::PutNumber(name, value);
}

inline void Log(std::string_view name, bool value)
{
    frc::SmartDashboard::PutBoolean(name, value);
}

inline void Log(std::string_view name, std::string_view value)
{
    frc::SmartDashboard::PutString(name, value);
}

inline void Log(std::string_view name, wpi::array<frc::SwerveModuleState, 4> value)
{
    static std::unordered_map<std::string_view, nt::StructArrayPublisher<frc::SwerveModuleState>> publishers;
    
    publishers.try_emplace(
        name,
        nt::NetworkTableInstance::GetDefault()
            .GetStructArrayTopic<frc::SwerveModuleState>(name)
            .Publish()
    );
    
    publishers[name].Set(value);
}

inline void Log(std::string_view name, wpi::array<frc::SwerveModulePosition, 4> value)
{
    static std::unordered_map<std::string_view, nt::StructArrayPublisher<frc::SwerveModulePosition>> publishers;
    
    publishers.try_emplace(
        name,
        nt::NetworkTableInstance::GetDefault()
            .GetStructArrayTopic<frc::SwerveModulePosition>(name)
            .Publish()
    );
    
    publishers[name].Set(value);
}

inline void Log(std::string_view name, frc::Pose2d value)
{
    static std::unordered_map<std::string_view, nt::StructPublisher<frc::Pose2d>> publishers;
    
    publishers.try_emplace(
        name,
        nt::NetworkTableInstance::GetDefault()
            .GetStructTopic<frc::Pose2d>(name)
            .Publish()
    );
    
    publishers[name].Set(value);
}

inline void Log(std::string_view name, frc::ChassisSpeeds value)
{
    static std::unordered_map<std::string_view, nt::StructPublisher<frc::ChassisSpeeds>> publishers;
    
    publishers.try_emplace(
        name,
        nt::NetworkTableInstance::GetDefault()
            .GetStructTopic<frc::ChassisSpeeds>(name)
            .Publish()
    );
    
    publishers[name].Set(value);
}