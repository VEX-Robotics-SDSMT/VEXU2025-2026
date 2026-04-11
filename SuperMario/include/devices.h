#pragma once
#include "pros/motor_group.hpp"

//===============================================
//Controller
//===============================================
inline pros::Controller master(pros::controller_id_e_t::E_CONTROLLER_MASTER);

//===============================================
//Ports
//===============================================

//Left Motors
const inline int8_t LM1 = 12;
const inline int8_t LM2 = 7;
const inline int8_t LM3 = 2;
const inline int8_t LM4 = 3;

//Right Motors
const inline int8_t RM1 = 6;
const inline int8_t RM2 = 10;
const inline int8_t RM3 = 1;
const inline int8_t RM4 = 9;

//Intake Motors
const inline int8_t IN1 = 20;
const inline int8_t IN2 = 21;

//Encoder
const inline int8_t ENC = 4;

//Inertial
const inline uint8_t INERT = 16;

//Pneumatics
const inline char HOOD_PORT = 'F';
const inline char ARM_PORT = 'H';
const inline char LIFT_PORT = 'G';


//===============================================
//Motor Groups
//===============================================

//Drive Train Motors
inline pros::MotorGroup leftMotors({-LM1, -LM2, -LM3, -LM4});
inline pros::MotorGroup rightMotors({RM1, RM2, RM3, RM4});

//Intake Group
inline pros::MotorGroup intakeMotors({IN1, IN2});

//===============================================
//Sensors
//===============================================

//Inertial Senor
inline pros::Imu imu(INERT);

//Rotational Senor
inline pros::Rotation enc(ENC);

//===============================================
//Pneumatics
//===============================================
inline pros::adi::Pneumatics hood(HOOD_PORT, false);
inline pros::adi::Pneumatics arm(ARM_PORT, false);
inline pros::adi::Pneumatics lift(LIFT_PORT, false);


//===============================================
//Drivetrain
//===============================================
inline Mines::Drivetrain drive(&leftMotors, &rightMotors, &enc, &imu);