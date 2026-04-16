#include "Drivetrain.h"

#include <numbers>

namespace Mines{
//=======================================
// Member Variables
//=======================================

//pros::MotorGroup m_leftMotors;
//pros::MotorGroup m_rightMotors;
//Mines::PID m_drivePID;
//Mines::PID m_turnPID;


Drivetrain::Drivetrain(pros::AbstractMotor* leftMotors,
                  pros::AbstractMotor* rightMotors,
                  pros::adi::Encoder* rotationSensor,
                  pros::Imu* imu,
                  double gearRatio, double wheelDiameter)
:
  m_drivePID(9, .001, .05, 20),
  m_turnPID(5, .001, .5, 20),
  m_gearRatio(gearRatio),
  m_wheelDiameter(wheelDiameter)
{
    m_leftMotors = leftMotors;
    m_rightMotors = rightMotors;
    m_rotation = rotationSensor;
    m_imu = imu;
}

void Drivetrain::initilizeDevices()
{
    m_imu->reset(true);
}

void Drivetrain::driveDistance(double distance, int32_t tol, uint32_t settleTime)
{
    double distancePerTurn = (m_wheelDiameter * std::numbers::pi) / m_gearRatio;
    double target = m_rotation->get_value() + (((distance / distancePerTurn) * 360 )); 

    m_drivePID.setTarget(target);

    uint32_t time = 0;
    while(time <= settleTime)
    {
        int32_t motorPower = std::clamp((int32_t)m_drivePID.calculate(static_cast<double>(m_rotation->get_value())),-m_voltageCap,m_voltageCap);
        m_leftMotors->move(motorPower);
        m_rightMotors->move(motorPower);

        if(m_rotation->get_value() <= target + tol && m_rotation->get_value() >= target - tol)
        {
            time += 5;
        }
        else
        {
            time = 0;
        }

        pros::delay(5);
    }
    m_leftMotors->move(0);
    m_rightMotors->move(0);

    return;
}

void Drivetrain::turnTo(double angle, double tol, uint32_t settleTime)
{
    double target = m_imu->get_rotation() + angle;

    m_turnPID.setTarget(target);

    uint32_t time = 0;

    while(time <= settleTime)
    {
        PRINT_D(m_imu->get_heading());
        PRINT_D(m_imu->get_rotation());
        int32_t motorPower = std::clamp((int32_t)m_turnPID.calculate(m_imu->get_rotation()),-m_voltageCap,m_voltageCap);
        m_leftMotors->move(motorPower);
        m_rightMotors->move(-motorPower);

        if(m_imu->get_rotation() <= target + tol && m_imu->get_rotation() >= target - tol)
        {
            time += 5;
        }
        else
        {
            time = 0;
        }

        pros::delay(5);
    }

    return;
}
}