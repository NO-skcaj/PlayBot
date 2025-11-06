#pragma once

#include "lib/hardware/gyro/Navx.h"

// Wrapper class for the Navx gyro sensor
class Gyro : public hardware::gyro::Navx, public frc2::SubsystemBase
{
    public:

        // Method that returns a pointer to the singleton instance of the Gyro class
        static Gyro* GetInstance()
        {
            // Singleton instance of the Gyro class
            static Gyro gyro;

            // Return the static gyro instance
            return &gyro;
        }

    private:

        // Private constructor for the Gyro class
        Gyro() : Navx()
        {

        }
};
