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

        const int COUNTER = (F_CPU / 64 * Ts_s) - 1;

        OCR1A = COUNTER;            
        TCCR1B |= (1 << WGM12);
        TCCR1B |= (1 << CS11) | (1 << CS10);
        
        TIMSK1 |= (1 << OCIE1A);

        interrupts();             
    }
    
    void INIT(){
        // leftEncoder.init();
        // rightEncoder.init();

        // leftMotor.init();
        // rightMotor.init();
        
        // TICK_TO_TIM();

        //cycloWorker.printCycloProgram();
    }

    void TICK(){
        // leftEncoder.tick();
        // rightEncoder.tick();
        
        // leftVelocityEstimator.tick();
        // rightVelocityEstimator.tick();
        
        // leftServo.tick();
        // rightServo.tick();
        
        // odometry.update(leftVelocityEstimator.getW(), rightVelocityEstimator.getW());

        // robot.moveFloodFill();

        // cycloWorker.doCyclogram();

        // static uint32_t timer = 0;

        // Serial.println(micros() - timer);

        // timer = micros();
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
            // maze.PrintDirPath();

            actionsHandler.primitivesToExplorers();
            cycloStore.printSmarts();
            
            // cycloStore.printSmarts();
        }
    }
}

// ISR(TIMER1_COMPA_vect){
//     DEVICES::TICK();   
// }