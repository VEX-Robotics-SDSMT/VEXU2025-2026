#include "PID.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

using namespace std;

namespace Mines
{
    PID::PID(const PIDTuning& tuning, double tolerance, time_t goalTime, time_t timeoutTime)
    : m_tuning(tuning), m_tolerance(tolerance), m_prevTime(pros::millis())
    {
        setTimeout(timeoutTime);
        setGoalTime(goalTime);
    }

    void PID::setTarget(double target)
    {
        m_target = target;
        m_timeSinceReached = 0;
        m_timeSinceSet = 0;
    }

    double PID::update(double measuredVal)
    {
        time_t currentTime = pros::millis();
        time_t deltaTime = currentTime - m_prevTime;
        
        double error = m_target - measuredVal;
        
        
        double proportion = m_tuning.kP * error;
        //TODO: add integral windup guard
        m_integral += m_tuning.kI * (error * (double)deltaTime);
        
        double derivative = 0.0;
        if(deltaTime > 0)
            derivative = m_tuning.kD * ( (error - m_prevError) / deltaTime);

        m_prevError = error;
        m_prevTime = currentTime;

        if(IN_RANGE(measuredVal, m_target - m_tolerance, m_target + m_tolerance))
        {
            m_timeSinceReached += deltaTime;
        }
        else
        {
            m_timeSinceReached = 0;
        }

        m_timeSinceSet += deltaTime;

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

    void PID::setGoalTime(time_t time)
    {
        m_goalTime = time;
    }

    void PID::setTimeout(time_t time)
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