#include "PID.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <algorithm>

using namespace std;

namespace Mines
{
    PID::PID(const PIDTuning& tuning)
    : m_tuning(tuning)
    {
    }

    void PID::reset()
    {
        m_integral = 0.0;
        m_prevError = 0.0;
    }

    double PID::update(double target, double measuredVal, double dt)
    {
        //calculate the error
        double error = target - measuredVal;
        

        //calculate the integral
        m_integral +=  error * dt;
        std::clamp(m_integral, -m_tuning.integralClamp, m_tuning.integralClamp);
        
        //calculate the derivative
        double derivative = 0.0;
        if(dt > 0)
            derivative = (error - m_prevError) / dt;



        //set the previous error and previoius time for next iteration
        m_prevError = error;
        
        double output = (m_tuning.kP * error) + (m_tuning.kI * m_integral) + (m_tuning.kD * derivative);
        std::clamp(output, m_tuning.minOutput, m_tuning.maxOutput);

        return output;
    }

    const PIDTuning &PID::getTuning()
    {
        return m_tuning;
    }
    void PID::setTuning(const PIDTuning &tuning)
    {
        m_tuning = tuning;
    }

}