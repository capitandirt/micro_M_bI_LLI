#include "Robot.h"


extern Encoder leftEncoder;
extern Encoder rightEncoder;

extern Motor leftMotor;
extern Motor rightMotor;

extern Servo leftServo;
extern Servo rightServo;

extern VelocityEstimator leftVelocityEstimator;
extern VelocityEstimator rightVelocityEstimator;

extern CycloStore cycloStore;
extern CycloWorker cycloWorker;

extern Maze maze;
extern Solver solver;
extern Robot robot;

namespace DEVICES{
    void TICK_TO_TIM(){
        noInterrupts();           
        TCCR1A = 0;               
        TCCR1B = 0;
    
        TCNT1 = 0;                
        
        const int PRESCALER = 64;
        const int COUNTER = F_CPU / PRESCALER * Ts_s - 1;

        OCR1A = 15624;            
        TCCR1B |= (1 << WGM12);
        TCCR1B |= (1 << CS12) | (1 << CS10);
        TIMSK1 |= (1 << OCIE1A);
    
        interrupts();             
    }
    
    void INIT(){
        leftEncoder.init();
        rightEncoder.init();

        leftMotor.init();
        rightMotor.init();
        
        TICK_TO_TIM();

        cycloStore.addSmart(SmartCycloAction_t::FWD);
        cycloStore.addSmart(SmartCycloAction_t::FWD);
        cycloStore.addSmart(SmartCycloAction_t::FWD);
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
        // robot.moveFloodFill();
    }

    namespace TEST{
        void SET_SERIAL(){
            Serial.begin(115200);
        }

        void BFS(){
            solver.MazeTestConfig();
            
            solver.SolveBfsMaze({0, 0}, {5, 5});
            
            maze.PrintDirPath();
            maze.Print();
        }
    
        void CYCLOGRAMS(){
            cycloStore.addSmart(SmartCycloAction_t::IDLE);
            cycloStore.addSmart(SmartCycloAction_t::FWD);
            cycloStore.addSmart(SmartCycloAction_t::SS90SL);
            cycloStore.addSmart(SmartCycloAction_t::SS90SR);
            cycloStore.addSmart(SmartCycloAction_t::STOP);

            cycloStore.addPrimitive(PrimitiveCycloAction_t::BLANK);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::STOP);

            cycloStore.printSmarts();
            cycloStore.printPrimitives();
        }

        void CONVERT_PATH_TO_CYCLOGRAMS(){
            solver.MazeTestConfig();
            solver.SolveBfsMaze({0, 0}, {0, 10});

            maze.Print();
            maze.PrintDirPath();
            
            robot.DirsToPrimitives(PrimitiveCycloAction_t::FORWARD);
            robot.primitivesToFasts();
            // cycloStore.printSmarts();
        }
    }
}

ISR(TIMER1_COMPA_vect){
    DEVICES::TICK();   
}