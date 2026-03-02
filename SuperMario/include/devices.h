#pragma once
#include "pros/motor_group.hpp"

//===============================================
//Ports
//===============================================

//Left Motors
const inline int8_t LM1 = 16;
const inline int8_t LM2 = 17;
const inline int8_t LM3 = 18;
const inline int8_t LM4 = 19;

//Right Motors
const inline int8_t RM1 = 4;
const inline int8_t RM2 = 5;
const inline int8_t RM3 = 6;
const inline int8_t RM4 = 7;

//Intake Motors
const inline int8_t IN1 = 8;
const inline int8_t IN2 = 9;

//Encoder
const inline int8_t ENC = 21;

//Inertial
const inline uint8_t INERT = 20;

//===============================================
//Motor Groups
//===============================================

//Drive Train Motors
const inline pros::MotorGroup leftMotors({-LM1, -LM2, -LM3, -LM4});
const inline pros::MotorGroup rightMotors({RM1, RM2, RM3, RM4});

//Intake Group
const inline pros::MotorGroup intakeMotors({IN1, IN2});

//===============================================
//Sensors
//===============================================

//Inertial Senor
const inline pros::Imu imu(INERT);

//Rotational Senor
const inline pros::Rotation enc(ENC);

//===============================================
//Pneumatics
//===============================================