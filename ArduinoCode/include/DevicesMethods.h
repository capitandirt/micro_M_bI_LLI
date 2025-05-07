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
        
        optocoupler.init();

        maze.PrimaryFill();
        maze.SetCell(START_CELL, START_ROBOT_COORDS);
        // maze.PassCell(START_ROBOT_COORDS);

        // maze.Print();

        // cycloStore.addSmart(SmartCycloAction_t::FWD);
        cycloStore.addSmart(SmartCycloAction_t::FWD_HALF);
        cycloWorker.init();
        
        // odometry.updateMazeCoords(START_ROBOT_DIRECTION);
        // TIM2::INIT();
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
        cycloWorker.doCyclogram();

        if(cycloWorker.isComplete() && !robot.checkFloodFill()){
            // maze.PassCell(odometry.getMazeCoords());
            robot.stepFloodFill();

            // maze.Print();
            // cycloStore.printPrimitives();
            // cycloStore.printSmarts();
        }

        cycloWorker.checkIsComplete();
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

        void EXPLORER_LEFT_RIGHT_SMARTS(){
            cycloStore.addSmart(SmartCycloAction_t::SS90EL);
            cycloStore.addSmart(SmartCycloAction_t::SS90ER);
        }
        void EXPLORER_FWD_3_SMARTS()
        {
            cycloStore.addSmart(SmartCycloAction_t::FWD);
            cycloStore.addSmart(SmartCycloAction_t::FWD);
            cycloStore.addSmart(SmartCycloAction_t::FWD);
        }
        
        void EXPLORER_CYC()
        {
            cycloStore.addSmart(SmartCycloAction_t::IP90L);
            cycloStore.addSmart(SmartCycloAction_t::IP90R);
            cycloStore.addSmart(SmartCycloAction_t::IP90R);
            cycloStore.addSmart(SmartCycloAction_t::IP180);
            cycloStore.addSmart(SmartCycloAction_t::SS90EL);
        }

        void CONVERT_PATH_TO_CYCLOGRAMS(){
            solver.MazeTestConfig();

            // unnamed namespace
            {
                Vec2 __s = {0, 0}; Vec2 __f = {2, 2};    
                solver.SolveBfsMaze(__s, __f);
            }

            maze.Print();

            {   
                Direction __sd = Direction::S; Vec2 __sv = {0, 0};
                Vec2 __v;
                actionsHandler.primitivesToExplorers(__sd);
            }

            maze.PrintDirPath();
            cycloStore.printPrimitives();
            cycloStore.printSmarts();
            cycloStore.reloadSmarts();
            // cycloStore.printSmarts();
        }

        void OPTOCOUPLERS_SENSE(){
            optocoupler.tick();
            optocoupler.printSense();
        }
        
        void OPTOCOUPLERS_MASK(){
            optocoupler.tick();
            optocoupler.printMask();
        }

        void OPTOCOUPLERS_CELL(){
            optocoupler.tick();
            optocoupler.printAbsCell();
        }
    }
}

// ISR(TIMER2_COMPA_vect){
//     DEVICES::TICK();   
// }