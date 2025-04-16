#ifndef _DEVICES_H_
#define _DEVICES_H_

#include "Config.h"
#include "Encoder.h"
#include "VelocityEstimator.h"
#include "Motor.h"
#include "Odometry.h"
#include "PiReg.h"
#include "Servo.h"
#include "Mixer.h"
#include "Maze.h"
#include "Solver.h"
#include "CycloWorker.h"
#include "Robot.h"

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
    leftEncoder.isr_callback();
}

void right_encoder_ISR()
{
    rightEncoder.isr_callback();
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

CycloWorkerConnectionParams cwcp{
    .mixer = &mixer
};

CycloWorker cycloWorker(&cwcp);

Maze maze;

Solver solver(&maze);

RobotConnectionParams rcp{
    ._cycloWorker = &cycloWorker,
    ._solver = &solver,
    ._Maze = &maze
};

Robot robot(&rcp);

namespace DEVICES{
    void INIT(){
        leftEncoder.init();
        rightEncoder.init();

        leftMotor.init();
        rightMotor.init();
        
        leftServo.init();
        leftServo.init();
    }

    void TICK(){
        leftEncoder.tick();
        rightEncoder.tick();

        leftVelocityEstimator.tick();
        rightVelocityEstimator.tick();

        leftServo.tick();
        rightServo.tick();

        cycloWorker.doCyclogram();
    }

    namespace TEST{
        void SET_SERIAL(){
            Serial.begin(115200);
        }

        void BFS(){
            SET_SERIAL();

            maze.PrimaryFill();
            maze.Print();
            
            solver.MazeTestConfig();
            
            maze.PrimaryFill();
            solver.SolveBfsMaze(0, 0, 5, 5);
            
            maze.PrintDirPath();
            maze.Print();
        }
    
        void CYCLOGRAMS(){
            SET_SERIAL();
            
            cycloWorker.addAction(SmartCycloAction_t::IDLE);
            cycloWorker.addAction(SmartCycloAction_t::FWD);
            cycloWorker.addAction(SmartCycloAction_t::SS90SL);
            cycloWorker.addAction(SmartCycloAction_t::SS90SR);
            cycloWorker.addAction(SmartCycloAction_t::STOP);
            cycloWorker.printCycloProgram();
        }

        void CONVERT_PATH_TO_CYCLOGRAMS(){
            SET_SERIAL();

            solver.MazeTestConfig();
            solver.SolveBfsMaze(0, 0, 5, 5);
            
            // maze.Print();
            // maze.PrintDirPath();

            
            robot.convertPathToCyclogram();
            cycloWorker.printCycloProgram();
        }
    }
}

#endif // !_DEVICES_H_
