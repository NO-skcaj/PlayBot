#include "lib/hardware/motors/TalonFX.h"
#include "lib/hardware/motors/SparkMax.h"

#include "lib/hardware/sensors/DigitalInput.h"

#include "Constants.h"

class Volcano : public frc2::SubsystemBase
{
public:

    explicit Volcano();

    void SetKicker(bool running);

    void SetIndexers(bool running);

    void SetFlywheel(bool running);

    bool GetKickSensor();

    bool IsFlywheelAtSpeed();

private:

    enum class FlywheelState
    {
        OFF,
        IDLE,
        RUNNING
    };

    hardware::motor::TalonFX  m_flywheelMotor;

    hardware::motor::SparkMax m_indexerMotors[2];

    hardware::motor::SparkMax m_kickMotor;

    hardware::sensor::DigitalInput m_ballSensor;

    FlywheelState m_flywheelState;
};