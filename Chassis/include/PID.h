#pragma once

#include <string>
#include <iostream>
#include "api.h"
#include <math.h>
#include "Logger.h"
#include "TaskBase.h"
#include "units.h"

#define IN_RANGE(x, min, max) (x >= min && x <= max)

namespace Mines
{
    struct PIDTuning
    {
        float kP = 0.0;
        float kI = 0.0;
        float kD = 0.0;
    };

    class PID
    {
    public:
        PID(const PIDTuning& tuning, double tolerance = 0.05, time_t goalTime = 10, time_t timeoutTime = 10, timeUnit unit = msec);
       
        /**
         * Set the target value for the PID loop
         */
        void setTarget(double target);
        /**
         * Update the PID loop
         * \returns The output of the PID operations, normaly use this for voltage input into a motor.
         */
        double update(double measuredVal);

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

        /**
         * Set the time the robot must be within the target to complete the movement
         * \param time the time the robot must be in the target
         * \param unit the units of time
         */
        void setGoalTime(time_t time, timeUnit unit = seconds);
        
        /**
         * Sets the acceptable margin of error.
         * If the tolerance is too low this can cause oscilations and the robot may not reach it's target.
         * \param tolerance the acceptable margin of error
         */
        void setTolerance(double tolerance) { m_tolerance = tolerance; };
        
        /**
         * Sets the time until the PID times out
         * \param time the time until the loop times out
         * \param unit the unit of time you are using
         */
        void setTimeout(time_t time, timeUnit unit = seconds);

        /**
         * check if the PID loop has reached it's target for the correct ammount of time
         * \returns if the target has been reached or not
         */
        bool targetReached();

        /**
         * check if the PID loop should continue running or not
         * \returns true if the target is not reached and it has not timed out
         */
        bool shouldRun();

        operator bool()
        {
            return shouldRun();
        }

    private:
        PIDTuning m_tuning;
        double m_target = 0.0;
        time_t m_prevTime = 0;
        double m_integral = 0.0;
        double m_prevError = 0.0;
        double m_tolerance = 0.0;
        time_t m_timeSinceSet = 0;
        time_t m_timeSinceReached = 0;
        time_t m_goalTime = 0;
        time_t m_timeoutTime = 0;
    };

}
