#include "RobotContainer.h"

// Reference to the RobotContainer singleton class
RobotContainer *RobotContainer::m_robotContainer = nullptr;

#pragma region GetInstance
/// @brief Method to return a pointer to the RobotContainer class.
/// @return Pointer to the RobotContainer class.
RobotContainer *RobotContainer::GetInstance()
{
    // Detrermine if the class has already been instantiated
    if (m_robotContainer == nullptr)
    {
        // Instantiate the class
        m_robotContainer = new RobotContainer();
    }

    // Return the class pointer
    return m_robotContainer;
}
#pragma endregion

#pragma region RobotContainer
/// @brief Method to configure the robot and SmartDashboard configuration.
RobotContainer::RobotContainer() : m_chassis{Chassis::GetInstance()}, m_volcano{Volcano::GetInstance()}
{
    // Configure the chassis default command
    m_chassis->SetDefaultCommand(ChassisDrive(m_chassis, GetChassisSpeeds()));

    // Configure the volcano default command
    m_volcano->SetDefaultCommand(VolcanoIndexerControl(m_volcano, [this] { return m_flywheelLimiter.Calculate(frc::ApplyDeadband(m_driveController.GetRightTriggerAxis(), constants::controller::FlywheelDeadZone)).value(); } ));

    // Configure the operator controller
    std::pair<Button, frc2::CommandPtr> runOnceControls[] =
    {
        {constants::controller::A,           ChassisZeroHeading(Gyro::GetInstance())},
        {constants::controller::B,           FlipFieldCentricity(m_chassis)},
        {constants::controller::RightBumper, VolcanoFlywheelOn(m_volcano)},
        {constants::controller::LeftBumper,  VolcanoFlywheelOff(m_volcano)}
    };

    // Configure the run-once controls
    for (auto& [button, command] : runOnceControls)
    {
        frc2::JoystickButton(&m_driveController, int(button)).OnTrue(std::move(command));
    }
}
#pragma endregion

#pragma region GetChassisSpeeds
/// @brief Method to return the chassis speeds based on joystick inputs.
/// @return The chassis speeds based on joystick inputs.
std::function<frc::ChassisSpeeds()> RobotContainer::GetChassisSpeeds()
{
    // Return the chassis speeds based on joystick inputs
    return [this] () -> frc::ChassisSpeeds
    {
        // Return the chassis speeds based on joystick inputs
        return frc::ChassisSpeeds{
            -constants::swerve::RobotSwerveConfig.maxSpeed           * frc::ApplyDeadband(m_driveController.GetRawAxis(1), constants::controller::TranslationDeadZone),
            -constants::swerve::RobotSwerveConfig.maxSpeed           * frc::ApplyDeadband(m_driveController.GetRawAxis(0), constants::controller::TranslationDeadZone),
             constants::swerve::RobotSwerveConfig.maxAngularVelocity * frc::ApplyDeadband(m_driveController.GetRawAxis(4), constants::controller::RotateDeadZone)
        };
    };
}
#pragma endregion

#pragma region GetExponentialValue
/// @brief Method to convert a joystick value from -1.0 to 1.0 to exponential mode.
/// @param joystickValue The raw joystick value.
/// @param exponent The exponential value.
/// @return The resulting exponential value.
double RobotContainer::GetExponentialValue(double joystickValue, double exponent)
{
    int    direction = (joystickValue < 0.0) ? -1 : 1;
    double absValue  = std::abs(joystickValue);
    double output    = std::pow(absValue, exponent) * direction;

    // Ensure the range of the output
    if (output < -1.0) output = -1.0;
    if (output > 1.0)  output =  1.0;

    // Return the output value
    return output;
}
#pragma endregion
