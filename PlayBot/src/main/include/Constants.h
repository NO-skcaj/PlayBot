#pragma once

#pragma region Includes
#include <array>
#include <numbers>

#include <units/base.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/voltage.h>

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
}
