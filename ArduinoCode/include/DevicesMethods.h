#include "Robot.h"
#include "TIM2_HANDLER.h"

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
extern Odometry odometry;
extern OptocouplerSensors optocoupler;
extern Robot robot;

namespace DEVICES{  
    void INIT(){
        leftEncoder.init();
        rightEncoder.init();

        leftMotor.init();
        rightMotor.init();
        
        // cycloStore.addSmart(SmartCycloAction_t::FWD);

        TIM2::INIT();
    }

    void TICK(){
        leftEncoder.tick();
        rightEncoder.tick();
        
        leftVelocityEstimator.tick();
        rightVelocityEstimator.tick();
        
        leftServo.tick();
        rightServo.tick();

        optocoupler.tick();
        
        odometry.update(leftVelocityEstimator.getW(), rightVelocityEstimator.getW());

        // cycloWorker.doCyclogram();
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

            maze.Clear();
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

            cycloStore.printPrimitives();
            cycloStore.printSmarts();

            cycloStore.reloadPrimitives();
            cycloStore.reloadSmarts();
        }

        void CONVERT_PATH_TO_CYCLOGRAMS(){
            solver.MazeTestConfig();

            // unnamed namespace
            {
                Vec2 __s = {0, 0}; Vec2 __f = {5, 5};    
                solver.SolveBfsMaze(__s, __f);
            }

            maze.Print();

            {   
                auto __sd = Direction::N; Vec2 __sv = {0, 0};
                Direction __d; Vec2 __v;
                actionsHandler.primitivesToExplorers(__sd, __sv, __d, __v);
            }

            cycloStore.printSmarts();
            cycloStore.reloadSmarts();
            // cycloStore.printSmarts();
        }

        void OPTOCOUPLER(){
            optocoupler.printSense();
            optocoupler.printMask();
            Serial.println();
        }
    }
}

ISR(TIMER2_COMPA_vect){
    DEVICES::TICK();   
}