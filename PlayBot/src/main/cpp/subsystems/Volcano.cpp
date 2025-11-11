#include "subsystems/Volcano.h"

#pragma region Volcano
/// @brief Constructor for the Volcano subsystem.
Volcano::Volcano()
    : m_flywheelMotor
      {
          constants::volcano::volcanoConfig.flywheelMotorCANid, constants::volcano::volcanoConfig.flywheelMotorConfig, frc::DCMotor::Falcon500()
      },
      m_indexerMotors
      {
          hardware::motor::SparkMax{constants::volcano::volcanoConfig.indexerMotorsCANid[0], constants::volcano::volcanoConfig.indexerMotorsConfig, frc::DCMotor::NEO()},
          hardware::motor::SparkMax{constants::volcano::volcanoConfig.indexerMotorsCANid[1], constants::volcano::volcanoConfig.indexerMotorsConfig, frc::DCMotor::NEO()},
          hardware::motor::SparkMax{constants::volcano::volcanoConfig.indexerMotorsCANid[2], constants::volcano::volcanoConfig.indexerMotorsConfig, frc::DCMotor::NEO()}
      },
      m_flywheelState{FlywheelState::OFF}
{
    // Initialize the flywheel motor to an OFF state
    m_flywheelMotor.SetReferenceState(0_V);
}
#pragma endregion

#pragma region SetFlywheel
/// @brief Method to set the flywheel state.
/// @param isRunning Boolean indicating whether the flywheel should be running or not.
void Volcano::SetFlywheel(bool isRunning)
{
    // State machine to handle the flywheel states
    switch (m_flywheelState)
    {
        case FlywheelState::RUNNING:
        {
            if (!isRunning)
            { 
                m_flywheelMotor.SetReferenceState(constants::volcano::volcanoConfig.flywheelIdleVoltage);
                m_flywheelState = FlywheelState::IDLE;
            } 
            else
            {
                m_flywheelMotor.SetReferenceState(0_V);
                m_flywheelState = FlywheelState::OFF;
            }
            break;
        }
        case FlywheelState::IDLE:
        {
            if (isRunning)
            {
                m_flywheelMotor.SetReferenceState(constants::volcano::volcanoConfig.flywheelOnVoltage);
                m_flywheelState = FlywheelState::RUNNING;
            }
            else
            {
                m_flywheelMotor.SetReferenceState(0_V);
                m_flywheelState = FlywheelState::OFF;
            }
            break;
        }
        case FlywheelState::OFF:
        {
            if (isRunning)
            {
                m_flywheelMotor.SetReferenceState(constants::volcano::volcanoConfig.flywheelOnVoltage);
                m_flywheelState = FlywheelState::RUNNING;
            }
            else
            {
                m_flywheelMotor.SetReferenceState(constants::volcano::volcanoConfig.flywheelIdleVoltage);
                m_flywheelState = FlywheelState::IDLE;
            }
            break;
        }
    }
}
#pragma endregion

#pragma region SetIndexer
/// @brief Method to set the indexer motors' speed.
// Not sure what this should be, so it will be between 0 and 12 volts, limit as needed DO NOT LIMIT IN IMPLEMENTATION, LIMIT HERE
void Volcano::SetIndexer(double speed)
{
    // Set all indexer motors to the same speed
    for (auto& motor : m_indexerMotors)
    {
        // Set the motor reference state
        motor.SetReferenceState(speed);
    }
}
#pragma endregion
