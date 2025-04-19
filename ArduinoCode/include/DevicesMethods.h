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

extern Encoder leftEncoder;
extern Encoder rightEncoder;

extern Motor leftMotor;
extern Motor rightMotor;

extern Servo leftServo;
extern Servo rightServo;

extern VelocityEstimator leftVelocityEstimator;
extern VelocityEstimator rightVelocityEstimator;

extern CycloWorker cycloWorker;

extern Maze maze;
extern Solver solver;
extern Robot robot;

namespace DEVICES{
    void INIT(){
        leftEncoder.init();
        rightEncoder.init();

        leftMotor.init();
        rightMotor.init();
        
        // cycloWorker.addAction(SmartCycloAction_t::FWD);
        // cycloWorker.addAction(SmartCycloAction_t::SS90EL);
        // cycloWorker.addAction(SmartCycloAction_t::SS90ER);
        //cycloWorker.printCycloProgram();
    }

    void TICK(){
        leftEncoder.tick();
        rightEncoder.tick();
        
        leftVelocityEstimator.tick();
        rightVelocityEstimator.tick();
        
        leftServo.tick();
        rightServo.tick();
        
        odometry.update(leftVelocityEstimator.getW(), rightVelocityEstimator.getW());
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
            solver.SolveBfsMaze({0, 0}, {5, 5});
            
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
            solver.SolveBfsMaze({0, 0}, {5, 5});
            
            // maze.Print();
            // maze.PrintDirPath();
            
            robot.convertPathToCyclogram();
            cycloWorker.printCycloProgram();
        }
    }
}