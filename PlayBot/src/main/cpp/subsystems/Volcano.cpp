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
    m_kickMotor.SetReferenceState(running ? constants::volcano::targetKickerSpeed : 0.0_tps);
}
#pragma endregion

#pragma region IndexBall(bool ruinning)
/// @brief Method to kick the ball to the flywheel.
/// @param running Whether the kicker is running or not.
void Volcano::SetIndexers(bool ruinning)
{
    for (auto& indexer : m_indexerMotors)
    {
        indexer.SetReferenceState(ruinning ? constants::volcano::targetIndexerSpeed : 0.0_tps);
    }
}
#pragma endregion

#pragma region SetFlywheel
/// @param targetSpeed - the speed we want it at
void Volcano::SetFlywheel(units::turns_per_second_t targetSpeed)
{
    m_flywheelMotor.SetReferenceState(targetSpeed);
    m_targetSpeed = targetSpeed;
}
#pragma endregion

#pragma region IsBallDetected
bool Volcano::IsBallDetected()
{
    // TODO: Check if the sensor returns true or false when a ball is detected
    return m_ballSensor.Get();
}
#pragma endregion

#pragma region IsFlywheelAtSpeed
bool Volcano::IsFlywheelAtSpeed()
{
    auto flyWheelSpeed = m_flywheelMotor.GetVelocity();
    // Check if the flywheel speed is within 5% of the target speed
    return std::abs(flyWheelSpeed.value() - m_targetSpeed.value()) <= m_targetSpeed.value() * constants::volcano::flywheelSpeedTolerance;
}
#pragma endregion

#pragma region Periodic
// This method will be called once per scheduler run
// Does logging
void Volcano::Periodic()
{
    Log("Flywheel Speed (TPS)",        m_flywheelMotor.GetVelocity().value());
    Log("Target Flywheel Speed (TPS)", m_targetSpeed.value());
    Log("Ball Detected: ",               IsBallDetected());
    Log("Flywheel At Speed: ",           IsFlywheelAtSpeed());
}
#pragma endregion