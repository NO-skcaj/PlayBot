#pragma once

#pragma region Includes
#include <frc/Timer.h>

#include <frc2/command/SubsystemBase.h>

#include <frc/util/Color.h>

#include <hal/AddressableLED.h>
#include <hal/FRCUsageReporting.h>
#include <hal/HALBase.h>
#include <hal/PWM.h>
#include <hal/Ports.h>
#include <wpi/StackTrace.h>

#include "frc/Errors.h"
#pragma endregion


enum class LEDType
{
    SOLID,
    BLINK,
    RAINBOW,
    GRADIENT,
    OFF
};

// Inhertits from HAL_AddressableLEDData and extends it a bit
class LEDData : public HAL_AddressableLEDData 
{

    public:

        LEDData()
        {
            r = 0.0;
            g = 0.0;
            b = 0.0;
        }

        LEDData(int _r, int _g, int _b) 
        {
            r = _r;
            g = _g;
            b = _b;
            padding = 0;
        }

        void SetLED(const frc::Color& color) 
        {
            this->r = color.red * 255;
            this->g = color.green * 255;
            this->b = color.blue * 255;
        }

};

template <int m_length>
class LED : public frc2::SubsystemBase
{

    public:

        explicit LED(int port) 
            : m_port{port},
              m_ledData{},
              m_ledDataGetter{[] () {}}
        {
            int32_t status = 0;

            auto stack = wpi::GetStackTrace(1);
            m_pwmHandle = HAL_InitializePWMPort(HAL_GetPort(port), stack.c_str(), &status);
            FRC_CheckErrorStatus(status, "Port {}", port);
            if (m_pwmHandle == HAL_kInvalidHandle) {
                return;
            }

            // Initialize the handle for the LEDs
            m_handle = HAL_InitializeAddressableLED(m_pwmHandle, &status);
            FRC_CheckErrorStatus(status, "Port {}", port);
            if (m_handle == HAL_kInvalidHandle) {
                HAL_FreePWMPort(m_pwmHandle);
            }
            
            // Set the length of the LED strip
            HAL_SetAddressableLEDLength(m_handle, m_length, &status);
            FRC_CheckErrorStatus(status, "Port {} length {}", m_port, m_length);

            // Interaction with hardware to set the color order
            HAL_SetAddressableLEDColorOrder(m_handle, HAL_AddressableLEDColorOrder::HAL_ALED_RGB, &status);
            FRC_CheckErrorStatus(status, "Port {} Color order RGB", m_port);

            Start();

            // Send usage report
            HAL_Report(HALUsageReporting::kResourceType_AddressableLEDs, port + 1);
        }

        // Destructor, called when the class is destroyed/taken out of scope
        ~LED()
        {
            Stop();
            HAL_FreeAddressableLED(m_handle);
            HAL_FreePWMPort(m_pwmHandle);
        }

        void SetPattern(LEDType type, std::vector<frc::Color> colors = {frc::Color::kWhite})
        {
            switch (type)
            {
                case LEDType::BLINK: 
                    m_ledDataGetter = [&] ()
                    {
                        auto numOfColors = colors.size();
                        int colorIndex = 0;
                        for (int i = 0; i < m_length; i++)
                        {
                            if (frc::GetTime().to<int>() % 2)
                            {
                                m_ledData[i].SetLED(colors[colorIndex]);
                                colorIndex = (colorIndex + 1) % numOfColors;
                            } else
                            {
                                m_ledData[i].SetLED(frc::Color::kBlack);
                            }
                        }
                    };
                    break;

                case LEDType::SOLID:
                    m_ledDataGetter = [&] ()
                    {
                        auto numOfColors = colors.size();
                        int colorIndex = 0;
                        for (int i = 0; i < m_length; i++)
                        {
                            m_ledData[i].SetLED(colors[colorIndex]);
                            colorIndex = (colorIndex + 1) % numOfColors;
                        }
                    };
                    break;

                case LEDType::RAINBOW:
                    m_ledDataGetter = [&] () 
                    {
                        // Get the hue per each LED
                        for (int i = 0; i < m_length; i++)
                        {
                            int hue = ((i * 180) / m_length) % 180;
                            m_ledData[i].SetLED(frc::Color::FromHSV(hue, 255, 128)); // These numbers are taken from an example
                        }
                    };
                    break;

                case LEDType::GRADIENT:
                    // Same as SOLID but it doesnt repeat, 
                    // transverses the whole strip with one iterations of the given colors

                    // Check if it's just the default color
                    if (!(colors[0] == frc::Color::kWhite) && !(colors.size() == 1))
                    {
                        m_ledDataGetter = [&] () {
                            int numOfColors = colors.size();
                            int ledsPerSegment = (m_length - 1) / (numOfColors - 1);

                            for (int i = 0; i < m_length; i++) 
                            {
                                int colorIndex = (i / ledsPerSegment) % numOfColors;
                                double t = std::fmod(i / static_cast<double>(ledsPerSegment), 1.0); // Floating point remainder

                                // Linear interpolation between the current color and the next color
                                frc::Color gradientColor{
                                    wpi::Lerp(colors[colorIndex].red,   colors[(colorIndex + 1) % numOfColors].red,   t),
                                    wpi::Lerp(colors[colorIndex].green, colors[(colorIndex + 1) % numOfColors].green, t),
                                    wpi::Lerp(colors[colorIndex].blue,  colors[(colorIndex + 1) % numOfColors].blue,  t)
                                };

                                m_ledData[i].SetLED(gradientColor);
                            }
                        };
                    } 
                    else
                    {
                        // If no color is given (kWhite is the default value)), just return rainbow
                        SetPattern(LEDType::RAINBOW);
                    }
                    break;

                default:
                    SetPattern(LEDType::SOLID, {frc::Color::kBlack});
                    break;

            }
        }

        void Periodic() override
        {
            m_ledDataGetter();

            int32_t status = 0;
            HAL_WriteAddressableLEDData(m_handle, m_ledData.data(), m_ledData.size(), // this may not work
                                        &status);
            FRC_CheckErrorStatus(status, "Port {}", m_port);
        }
        
    private:

        void Start() 
        {
            int32_t status = 0;
            HAL_StartAddressableLEDOutput(m_handle, &status);
            FRC_CheckErrorStatus(status, "Port {}", m_port);
        }

        // Do not use this to set to black
        void Stop() 
        {
            int32_t status = 0;
            HAL_StopAddressableLEDOutput(m_handle, &status);
            FRC_CheckErrorStatus(status, "Port {}", m_port);
        }

        // Hardware handling
        int m_port;
        int m_handle    = HAL_kInvalidHandle; // Placeholders
        int m_pwmHandle = HAL_kInvalidHandle;

        // LED data as an array to force the correct length
        std::function<void(void)> m_ledDataGetter;
        std::array<LEDData, m_length>                  m_ledData;
};