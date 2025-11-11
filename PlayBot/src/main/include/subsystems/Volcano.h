#include "lib/hardware/motors/TalonFX.h"
#include "lib/hardware/motors/SparkMax.h"

#include "Constants.h"

class Volcano : public frc2::SubsystemBase
{
public:

    explicit Volcano();

    // Double tapping either buttons will toggle between RUNNING or IDLE and off, single tap will run/stop based on which button you choose
    // ex: given its on idle, you press the idle button again and it will turn off
    // ex: given its off, you press the run (or idle) button and it will turn on to run (or idle) speed
    // the boolean is an option between running or idle
    void SetFlywheel(bool isRunning);

    void SetIndexer(double speed);

private:

    enum class FlywheelState
    {
        OFF,
        IDLE,
        RUNNING
    };

    hardware::motor::TalonFX  m_flywheelMotor;

    hardware::motor::SparkMax m_indexerMotors[3];

    FlywheelState m_flywheelState;
};