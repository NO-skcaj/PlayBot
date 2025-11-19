#include "subsystems/Volcano.h"

#pragma region Volcano
/// @brief Constructor for the Volcano subsystem.
Volcano::Volcano()
{

}
#pragma endregion

#pragma region KickBall(bool ruinning)
/// @brief Method to kick the ball to the flywheel.
/// @param running Whether the kicker is running or not.
void Volcano::SetKicker(bool running)
{
    m_kickMotor.SetReferenceState(running ? 1.0 : 0.0); // TODO: REPLACE WITH REVOLUTION SETPOINT
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

#pragma region SetFlywheel
/// @param targetSpeed - the speed we want it at
void Volcano::SetFlywheel(units::turns_per_second_t targetSpeed)
{
    m_flywheelMotor.SetReferenceState(targetSpeed); // This is alright
    m_targetSpeed = targetSpeed;
}
#pragma endregion

#pragma region IsBallDetected
bool Volcano::IsBallDetected()
{
    return (bool) m_ballSensor;
}
#pragma endregion

#pragma region IsFlywheelAtSpeed
bool Volcano::IsFlywheelAtSpeed()
{
    auto flyWheelSpeed = m_flywheelMotor.GetVelocity();
    // Check if the flywheel speed is within 5% of the target speed
    return std::abs(flyWheelSpeed.value() - m_targetSpeed.value()) <= m_targetSpeed.value() * 0.05;
}
#pragma endregion