#pragma once

#pragma region Includes
#include <array>
#include <numbers>

#include <units/base.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/voltage.h>

#include <frc/geometry/Transform3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include "lib/hardware/motors/Motor.h"
#pragma endregion

typedef int Button;

namespace constants
{
    namespace swerve
    {
        struct SwerveConfiguration
        {
            // Motor and encoders CAN IDs
            CANid_t frontLeftDriveCANid;
            CANid_t frontLeftTurnCANid;
            CANid_t frontLeftEncoderCANid;

            CANid_t frontRightDriveCANid;
            CANid_t frontRightTurnCANid;
            CANid_t frontRightEncoderCANid;

            CANid_t backLeftDriveCANid;
            CANid_t backLeftTurnCANid;
            CANid_t backLeftEncoderCANid;

            CANid_t backRightDriveCANid;
            CANid_t backRightTurnCANid;
            CANid_t backRightEncoderCANid;

            // PID, feedforward, and other configurations for the motors
            hardware::motor::MotorConfiguration driveMotorConfig;
            hardware::motor::MotorConfiguration turnMotorConfig;

            // All encoders are going to be slightly off, this corrects that
            units::radian_t frontLeftForwardAngle;
            units::radian_t frontRightForwardAngle;
            units::radian_t rearLeftForwardAngle;
            units::radian_t rearRightForwardAngle;

            // These make sure to limit how fast the robot can go
            units::meters_per_second_t  maxSpeed;
            units::radians_per_second_t maxAngularVelocity;

            // Conversion factors for the motors from encoders to actual movement
            units::meter_t  driveConversion;
            units::radian_t angleConversion;

            // The physical dimensions of the robot
            units::meter_t wheelBase;
            units::meter_t trackWidth;
        };

        constexpr SwerveConfiguration RobotSwerveConfig
        {
            // CAN IDs
            1, 2, 11,  // Front Left Drive,  Turn, Encoder
            3, 4, 12,  // Front Right Drive, Turn, Encoder
            5, 6, 13,  // Back Left Drive,   Turn, Encoder
            7, 8, 14,  // Back Right Drive,  Turn, Encoder

            // Drive Motor Config
            hardware::motor::MotorConfiguration
            {
                40_A,            // Current Limit
                true,            // Brake Mode
                0.1, 0.02, 0.0,  // P, I , D
                0.0, 0.0, 0.0    // S, V, A
            },

            // Turn Motor Config
            hardware::motor::MotorConfiguration
            {
                30_A,           // Current Limit
                true,           // Brake Mode
                1.0, 0.0, 0.2,  // P, I , D
                0.0, 0.0, 0.0   // S, V, A
            },

            // Forward Angles
            units::radian_t{-0.193604 * 2 * std::numbers::pi},
            units::radian_t{-0.422119 * 2 * std::numbers::pi},
            units::radian_t{-0.174561 * 2 * std::numbers::pi},
            units::radian_t{ 0.268555 * 2 * std::numbers::pi},

            4_mps,                                                 // Max Speed 
            units::radians_per_second_t{2 * std::numbers::pi},     // Max Angular Velocity

            units::meter_t{(0.0098022 * std::numbers::pi) / 6.75}, // Drive Conversion
            units::radian_t{(2 * std::numbers::pi) / 21.5},        // Angle Conversion

            // Robot Dimensions 0.762 meters ^ 2
            25_in, // Wheelbase
            25_in  // Trackwidth
        };
    }

    namespace volcano
    {
        struct VolcanoConfiguration
        {
            // CAN IDs
            CANid_t                             flywheelMotorCANid;
            std::array<CANid_t, 3>              indexerMotorsCANid;

            // Motor Configurations
            hardware::motor::MotorConfiguration flywheelMotorConfig;
            hardware::motor::MotorConfiguration indexerMotorsConfig;

            units::volt_t                       flywheelIdleVoltage;
            units::volt_t                       flywheelOnVoltage;
        };

        constexpr VolcanoConfiguration volcanoConfig
        {
            // CAN IDs
              20,           // Flywheel Motor
            { 21, 22, 23 }, // Indexer Motors

            // Motor Configurations
            // Flywheel Motor Config, do not use PIDSVA, use voltage
            hardware::motor::MotorConfiguration
            {
                30_A,           // Current Limit
                false,          // Brake Mode
                0.0, 0.0, 0.0,  // P, I , D
                0.0, 0.2, 0.0   // S, V, A
            },

            // Indexer Motors Config
            hardware::motor::MotorConfiguration
            {
                30_A,           // Current Limit
                true,           // Brake Mode
                1.0, 0.0, 0.0,  // P, I , D
                0.0, 0.0, 0.0   // S, V, A
            },

            // Flywheel Voltages
            2_V,  // Idle Voltage
            10_V  // On Voltage
        };
    }

    namespace controller
    {
        // Drive Input Configurations
        constexpr int    DrivePort           =    0;

        constexpr double TranslationDeadZone = 0.01;
        constexpr double RotateDeadZone      = 0.01;
        constexpr double FlywheelDeadZone    = 0.01;

        constexpr double ExponentForward     = 3.0;
        constexpr double ExponentStrafe      = 3.0;
        constexpr double ExponentAngle       = 3.0;

        // BUTTONSSSSS
        constexpr Button A                   =   1;
        constexpr Button B                   =   2;
        constexpr Button X                   =   3;
        constexpr Button Y                   =   4;
        constexpr Button LeftBumper          =   5;
        constexpr Button RightBumper         =   6;
        constexpr Button Back                =   7;
        constexpr Button Start               =   8;
        constexpr Button LeftStickButton     =   9;
        constexpr Button RightStickButton    =  10;

        constexpr Button Pov_0               =   0;
        constexpr Button Pov_45              =  45;
        constexpr Button Pov_90              =  90;
        constexpr Button Pov_135             = 135;
        constexpr Button Pov_180             = 180;
        constexpr Button Pov_225             = 225;
        constexpr Button Pov_270             = 270;
        constexpr Button Pov_315             = 315;
    }

    namespace vision 
    {
        constexpr std::string_view            CameraName{"PhotonCamera"};

        constexpr frc::Transform3d            RobotToCam{frc::Translation3d{0_m, 4_in, 15_in}, frc::Rotation3d{}};

        const     frc::AprilTagFieldLayout    TagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeAndyMark);

        const     Eigen::Matrix<double, 3, 1> SingleTagStdDevs{4, 4, 8};

        const     Eigen::Matrix<double, 3, 1> MultiTagStdDevs{0.5, 0.5, 1};

        namespace AprilTagLocations
        {
            // constexpr frc::Pose3d One      {657.37_in,  25.80_in, 58.50_in, {126_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Two      {657.37_in, 291.20_in, 58.50_in, {234_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Three    {455.15_in, 317.15_in, 51.25_in, {270_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Four     {365.20_in, 241.64_in, 73.54_in, {  0_deg, 0_deg, 30_deg}};
            // constexpr frc::Pose3d Five     {365.20_in,  75.39_in, 73.54_in, {  0_deg, 0_deg, 30_deg}};
            // constexpr frc::Pose3d Six      {530.49_in, 130.17_in, 12.13_in, {300_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Seven    {546.87_in, 158.50_in, 12.13_in, {  0_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Eight    {530.49_in, 186.83_in, 12.13_in, { 60_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Nine     {497.77_in, 186.83_in, 12.13_in, {120_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Ten      {481.39_in, 158.50_in, 12.13_in, {180_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Eleven   {497.77_in, 130.17_in, 12.13_in, {240_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Twelve   {33.51_in,   25.80_in, 58.50_in, { 54_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Thirteen {33.51_in,  291.20_in, 58.50_in, {306_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Fourteen {325.68_in, 241.64_in, 73.54_in, {180_deg, 0_deg, 30_deg}};
            // constexpr frc::Pose3d Fifteen  {325.68_in,  75.39_in, 73.54_in, {180_deg, 0_deg, 30_deg}};
            // constexpr frc::Pose3d Sixteen  {235.73_in,  -0.15_in, 51.25_in, { 90_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Seventeen{160.39_in, 130.17_in, 12.13_in, {240_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Eighteen {144.00_in, 158.50_in, 12.13_in, {180_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Nineteen {160.39_in, 186.83_in, 12.13_in, {120_deg, 0_deg,  0_deg}};
            // constexpr frc::Pose3d Twenty   {193.10_in, 186.83_in, 12.13_in, {60_deg,  0_deg,  0_deg}};
            // constexpr frc::Pose3d Twentyone{209.49_in, 158.50_in, 12.13_in, {0_deg,   0_deg,  0_deg}};
            // constexpr frc::Pose3d Twentytwo{193.10_in, 130.17_in, 12.13_in, {300_deg, 0_deg,  0_deg}};

            constexpr frc::Pose2d Tags2d[22] = 
            { 
                { 657.37_in,  25.80_in, { 126_deg} }, { 657.37_in, 291.20_in, { 234_deg} },
                { 455.15_in, 317.15_in, { 270_deg} }, { 365.20_in, 241.64_in, {   0_deg} },
                { 365.20_in,  75.39_in, {   0_deg} }, { 530.49_in, 130.17_in, { 300_deg} },
                { 546.87_in, 158.50_in, {   0_deg} }, { 530.49_in, 186.83_in, {  60_deg} },
                { 497.77_in, 186.83_in, { 120_deg} }, { 481.39_in, 158.50_in, { 180_deg} },
                { 497.77_in, 130.17_in, { 240_deg} }, { 33.51_in,   25.80_in, {  54_deg} },
                { 33.51_in,  291.20_in, { 306_deg} }, { 325.68_in, 241.64_in, { 180_deg} },
                { 325.68_in,  75.39_in, { 180_deg} }, { 235.73_in,  -0.15_in, {  90_deg} },
                { 160.39_in, 130.17_in, { 240_deg} }, { 144.00_in, 158.50_in, { 180_deg} },
                { 160.39_in, 186.83_in, { 120_deg} }, { 193.10_in, 186.83_in, { 60_deg,} },
                { 209.49_in, 158.50_in, { 0_deg, } }, { 193.10_in, 130.17_in, { 300_deg} }
            };

            constexpr std::span<const frc::Pose2d> Pose2dTagsSpan{std::begin(Tags2d), std::end(Tags2d)};
        }
    }
}
