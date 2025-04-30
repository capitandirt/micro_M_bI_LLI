#ifndef _DEVICES_H_
#define _DEVICES_H_

#include "Robot.h"
#include "CycloWorker.h"

void left_encoder_ISR();
void right_encoder_ISR();

EncoderConnectionParams left_ecp {
    .CLK_A_PIN = LEFT_CLK_A_PIN,
    .B_PIN = LEFT_B_PIN,
    .ENC_DIR = LEFT_ENC_DIR,
    .ISR = left_encoder_ISR
};
Encoder leftEncoder(&left_ecp);

EncoderConnectionParams right_ecp {
    .CLK_A_PIN = RIGHT_CLK_A_PIN,
    .B_PIN = RIGHT_B_PIN,
    .ENC_DIR = RIGHT_ENC_DIR,
    .ISR = right_encoder_ISR
};

Encoder rightEncoder(&right_ecp);

void left_encoder_ISR()
{
    leftEncoder.isrCallback();
}

void right_encoder_ISR()
{
    rightEncoder.isrCallback();
}

VelocityEstimatorConnectionParams left_vecp {
    .encoder = &leftEncoder
};

VelocityEstimator leftVelocityEstimator(&left_vecp);

VelocityEstimatorConnectionParams right_vecp {
    .encoder = &rightEncoder
};

VelocityEstimator rightVelocityEstimator(&right_vecp);

Odometry odometry;

MotorConnectionParams left_mcp {
    .DIR = LEFT_MOTOR_DIR,
    .PWM = LEFT_MOTOR_PWM,
    .M_POLARITY = LEFT_MOTOR_POLARITY
};
Motor leftMotor(&left_mcp);

MotorConnectionParams right_mcp {
    .DIR = RIGHT_MOTOR_DIR,
    .PWM = RIGHT_MOTOR_PWM,
    .M_POLARITY = RIGHT_MOTOR_POLARITY
};
Motor rightMotor(&right_mcp);

PiRegConnectionParams left_w_prcp{
    .Kp = W_KP,
    .Ki = W_KI
};
PiReg left_w_PiReg(&left_w_prcp);

PiRegConnectionParams right_w_prcp{
    .Kp = W_KP,
    .Ki = W_KI
};
PiReg right_w_PiReg(&right_w_prcp);

ServoConnectionParams left_scp{
    .w_PiReg = &left_w_PiReg,
    .motor = &leftMotor,
    .velocityEstimator = &leftVelocityEstimator
};
Servo leftServo(&left_scp);

ServoConnectionParams right_scp{
    .w_PiReg = &right_w_PiReg,
    .motor = &rightMotor,
    .velocityEstimator = &rightVelocityEstimator
};
Servo rightServo(&right_scp);

MotionControlConnectionParams mccp{
    .leftServo = &leftServo,
    .rightServo = &rightServo
};
Mixer mixer(&mccp);

CycloStore cycloStore;

CycloWorkerConnectionParams cwcp{
    .mixer = &mixer,
    .odometry = &odometry,
    .cycloStore = &cycloStore
};
CycloWorker cycloWorker(&cwcp);

Maze maze;

Solver solver(&maze);

ActionsHandlerConnectionParams ahcp{
    ._maze = &maze,
    ._cycloStore = &cycloStore
};
ActionsHandler actionsHandler(&ahcp);

OprocouplerConnectionParams ocp{
    .EMITERS_FWD = OPTOCOUPLER_EMITERS_A,
    .EMITERS_SIDE = OPTOCOUPLER_EMITERS_B,
    .REC_RIGHT = OPTOCOUPLER_SENSOR_1,
    .REC_LEFT = OPTOCOUPLER_SENSOR_2,
    .REC_FWD_LEFT = OPTOCOUPLER_SENSOR_3,
    .REC_FWD_RIGHT = OPTOCOUPLER_SENSOR_0,
    .SENSE_THRESHOLD_FWD_L = OPTOCOUPLER_SENSE_THRESHOLD_FWD_L,
    .SENSE_THRESHOLD_FWD_R = OPTOCOUPLER_SENSE_THRESHOLD_FWD_R,
    .SENSE_THRESHOLD_RIGHT = OPTOCOUPLER_SENSE_THRESHOLD_RIGHT,
    .SENSE_THRESHOLD_LEFT = OPTOCOUPLER_SENSE_THRESHOLD_LEFT
};
OptocouplerSensors optocoupler(&ocp);

RobotConnectionParams rcp{
    ._actionsHandler = &actionsHandler,
    ._maze = &maze,
    ._solver = &solver,
    ._optocoupler = &optocoupler,
    ._odometry = &odometry
};
Robot robot(&rcp);

#endif // !_DEVICES_H_
