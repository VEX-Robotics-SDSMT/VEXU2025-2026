#include "PID.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <algorithm>

using namespace std;

namespace Mines
{
    PID::PID(const PIDTuning& tuning, double tolerance, uint32_t goalTime, uint32_t timeoutTime)
    : m_tuning(tuning), m_tolerance(tolerance), m_prevTime(pros::millis())
    {
        setTimeout(timeoutTime);
        setGoalTime(goalTime);
    }

    void PID::setTarget(double target)
    {
        m_target = target;
        m_timeSinceReached = 0.0;
        m_timeSinceSet = 0.0;
        m_integral = 0.0;
        m_prevError = 0.0;
    }

    double PID::update(double measuredVal)
    {
        uint32_t currentTime = pros::millis();
        //get the delta time in seconds
        double deltaTime = (currentTime - m_prevTime) / 1000.0;
        
        //calculate the error
        double error = m_target - measuredVal;
        
        //calulate the proportion
        double proportion = m_tuning.kP * error;

        //calculate the integral
        m_integral += m_tuning.kI * (error * deltaTime);
        std::clamp(m_integral, -m_tuning.maxI, m_tuning.maxI);
        
        //calculate the derivative
        double derivative = 0.0;
        if(deltaTime > 0)
            derivative = m_tuning.kD * ( (error - m_prevError) / deltaTime);


        //accumulate time since the target was reached
        if(IN_RANGE(measuredVal, m_target - m_tolerance, m_target + m_tolerance))
        {
            m_timeSinceReached += deltaTime;
        }
        else
        {
            m_timeSinceReached = 0;
        }

        m_timeSinceSet += deltaTime;

        //set the previous error and previoius time for next iteration
        m_prevError = error;
        m_prevTime = currentTime;

        return proportion + m_integral + derivative;
    }

    const PIDTuning &PID::getTuning()
    {
        return m_tuning;
    }
    void PID::setTuning(const PIDTuning &tuning)
    {
        m_tuning = tuning;
    }

    void PID::setGoalTime(double time)
    {
        m_goalTime = time;
    }

    void PID::setTimeout(double time)
    {
        m_timeoutTime = time;
    }

    bool PID::targetReached()
    {
        return m_timeSinceReached >= m_goalTime;
    }

    bool PID::shouldRun()
    {
        return (!targetReached()) && m_timeSinceSet < m_timeoutTime;
    }
}