#include "subsystems/Volcano.h"

#pragma region Volcano
/// @brief Constructor for the Volcano subsystem.
Volcano::Volcano()
    : m_flywheelMotor
      {
          constants::volcano::flywheelMotorCANid, constants::volcano::flywheelMotorConfig, frc::DCMotor::Falcon500()
      },
      m_indexerMotors
      {
          hardware::motor::SparkMax{
            constants::volcano::firstIndexerMotorCANid,  constants::volcano::indexerMotorConfig, frc::DCMotor::NEO()},
          hardware::motor::SparkMax{
            constants::volcano::secondIndexerMotorCANid, constants::volcano::indexerMotorConfig, frc::DCMotor::NEO()}
      },
      m_kickMotor{constants::volcano::kickerMotorCANid, constants::volcano::kickMotorConfig, frc::DCMotor::NEO()},

      m_ballSensor{constants::volcano::ballSensorDIOPort},
      
      m_flywheelState{FlywheelState::OFF}
{}
#pragma endregion

#pragma region KickBall(bool ruinning)
/// @brief Method to kick the ball to the flywheel.
/// @param running Whether the kicker is running or not.
void Volcano::SetKicker(bool running)
{
    m_kickMotor.SetReferenceState(running ? 1.0 : 0.0); // REPLACE WITH REVOLUTION SETPOINT
}
#pragma endregion

#pragma region IndexBall(bool ruinning)
/// @brief Method to kick the ball to the flywheel.
/// @param running Whether the kicker is running or not.
void Volcano::SetIndexers(bool ruinning)
{
    for (auto& indexer : m_indexerMotors)
    {
        indexer.SetReferenceState(ruinning ? 1.0 : 0.0); // REPLACE WITH REVOLUTION SETPOINT
    }
}
#pragma endregion

#pragma region SpinUpFlyWheel(bool running)
void Volcano::SetFlywheel(bool running)
{
    m_flywheelMotor.SetReferenceState(running ? 1.0 : 0.0); // This is alright
}
#pragma endregion

#pragma region GetKickSensor
bool Volcano::GetKickSensor()
{
    return (bool) m_ballSensor;
}
#pragma endregion

#pragma region IsFlywheelAtSpeed
bool Volcano::IsFlywheelAtSpeed()
{
    return m_flywheelMotor.GetVelocity() >= 10_tps; // arbitrary, this number doesn't mean anything
}
#pragma endregion