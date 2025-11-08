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

    hardware::motor::TalonFX  m_flywheelMotor
    {
        constants::volcano::volcanoConfig.flywheelMotorCANid, constants::volcano::volcanoConfig.flywheelMotorConfig, frc::DCMotor::Falcon500()
    };

    hardware::motor::SparkMax m_indexerMotors[3]
    {
        hardware::motor::SparkMax{constants::volcano::volcanoConfig.indexerMotorsCANid[0], constants::volcano::volcanoConfig.indexerMotorsConfig, frc::DCMotor::NEO()},
        hardware::motor::SparkMax{constants::volcano::volcanoConfig.indexerMotorsCANid[1], constants::volcano::volcanoConfig.indexerMotorsConfig, frc::DCMotor::NEO()},
        hardware::motor::SparkMax{constants::volcano::volcanoConfig.indexerMotorsCANid[2], constants::volcano::volcanoConfig.indexerMotorsConfig, frc::DCMotor::NEO()}
    };

    FlywheelState m_flywheelState = FlywheelState::OFF;
};