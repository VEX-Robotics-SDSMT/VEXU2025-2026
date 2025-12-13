#pragma once

#include <cstdint>

namespace Mines
{
    struct PIDTuning
    {
        //maximum range for the integral value to prevent windup default to +/-10
        double integralClamp = 10.0;
        double maxOutput = 127.0;
        double minOutput = -127.0;
        //PID constants
        double kP = 0.0;
        double kI = 0.0;
        double kD = 0.0;
    };

    class PID
    {
    public:
        PID(const PIDTuning& tuning);

        //Resets the internal state of the PID controller
        void reset();

        /**
         * Update the PID loop
         * \returns The output of the PID operations, normaly use this for voltage input into a motor.
         */
        double update(double target, double measuredVal, double dt);

        /**
         * Gets the PID tuning constants
         * \returns PIDTuning reference that holds the constants
         */
        const PIDTuning& getTuning();
        /**
         * Sets the PID tuning constants
         * \param tuning the tuning values for the PID loop
         */
        void setTuning(const PIDTuning& tuning);


    private:
        PIDTuning m_tuning;
        double m_integral = 0.0;
        double m_prevError = 0.0;
    };

}