#include "PID.h"
#include "display/lv_misc/lv_task.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <algorithm>

using namespace std;

namespace Mines
{
    PID::PID(PIDInterface *inputInterface, LoggerSettings settings) : logger(settings)
    {
        interface = inputInterface;
    }

    void PID::update(double deltaT)
    {
        logger.Log("current target: " + std::to_string(GetTarget()), 0, LoggerSettings::verbose);
        double currentPosition = getPosition();
        logger.Log("current position: " + std::to_string(currentPosition), 1, LoggerSettings::verbose);
        double error = getError(target);
        logger.Log("current error value: " + std::to_string(error), 2, LoggerSettings::verbose);

        double positional = KP * error;
        lastIntegral += error * deltaT;
        double integral = KI * lastIntegral;
        double derivative = 0;
        if (deltaT > 0 && !std::isnan(deltaT)) {
            derivative = KD * ((error - lastError) / deltaT);
}
        logger.Log("positional: " + std::to_string(positional), 3, LoggerSettings::verbose);
        logger.Log("integral: " + std::to_string(integral), 4, LoggerSettings::verbose);
        logger.Log("derivative: " + std::to_string(derivative), 5, LoggerSettings::verbose);

        double controlVariable = (positional + integral + derivative);
        controlVariable = std::clamp(controlVariable, -1000.0, 1000.0);
        logger.Log("controlVariable: " + std::to_string(controlVariable), 7, LoggerSettings::verbose);


        //setting loop variables
        if (!isnan(error))
        {
            logger.Log("error: " + std::to_string(error), 8, LoggerSettings::verbose); 
            lastError = error;
        }
        else
        {
            logger.Log("ERROR: error is Nan", 8, LoggerSettings::error);
        }
        
        if (isnan(integral))
        {
            logger.Log("ERROR: integral is Nan", 9, LoggerSettings::error);
            lastIntegral = 0; 
        }

        //updating times
        timeSinceTargetSet += deltaT;
        if(fabs(error) < tolerance)
        {
            timeSinceTargetReached += deltaT;
        }

        //setting output variables
        //std::cout << controlVariable << endl;
        setOutput(controlVariable);
    }

    double PID::getPosition()
    {
        return interface->getPositionPID();
    }

    double PID::getError(double target) {
        return interface->getErrorPID(target);
    }

    void PID::setOutput(double value)
    {
        velocity = value;
        interface->setVelocityPID(value);
    }

    //----------------Getters/Setters-------------------
    void PID::resetTimers()
    {
        timeSinceTargetReached = 0;
        timeSinceTargetSet = 0;
    }

    void PID::SetPIDConst(double kp, double ki, double kd)
    {
        KP = kp;
        KI = ki;
        KD = kd;
    }

    void PID::SetTolerance(double tolerance)
    {
        this->tolerance = tolerance;
    }

    void PID::SetTarget(double target)
    {
        this->target = target;
        this->timeSinceTargetReached = 0;
        this->timeSinceTargetSet = 0;
    }

    double PID::GetVelocity()
    {
        return velocity;
    }

    double PID::GetTimeSinceTargetReached()
    {
        return timeSinceTargetReached;
    }

    double PID::GetTimeSinceTargetSet()
    {
        return timeSinceTargetSet;
    }

    double PID::GetTarget()
    {
        return target;
    }
}