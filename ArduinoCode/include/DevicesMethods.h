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


ISR(TIMER2_COMPA_vect){
    optocoupler.tick();
}

namespace DEVICES{  
    void INIT(){
        leftEncoder.init();
        rightEncoder.init();

        leftMotor.init();
        rightMotor.init();
        
        functionalSelector.init();
        optocoupler.init();

        // robot.init();

        TIM2::INIT();
    }

    void TICK(){
        leftEncoder.tick();
        rightEncoder.tick();
        
        leftVelocityEstimator.tick();
        rightVelocityEstimator.tick();
        
        // leftServo.setW(1);
        // rightServo.setW(1);

        leftServo.tick();
        rightServo.tick();

        // Serial.print(leftServo.velocityEstimator->getW());
        // Serial.print(' ');
        // Serial.println(rightServo.velocityEstimator->getW());
        
        // functionalSelector.tick();



        odometry.update(leftVelocityEstimator.getW(), rightVelocityEstimator.getW());

        cycloWorker.doCyclogram();

        // robot.stepFloodFill();
        
        cycloWorker.tryComplete();
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
    
        void UNDEF_CELL_WALLS(){
            maze.PrimaryFill();

            maze.SetCell({WallState::HI, WallState::HI, WallState::UNDEF, WallState::UNDEF}, {1, 1});
            maze.SetCell({WallState::UNDEF, WallState::UNDEF, WallState::HI, WallState::UNDEF}, {2, 1});
            maze.SetCell({WallState::LO, WallState::LO, WallState::LO, WallState::LO}, {0, 0});

            solver.SolveBfsMaze({0, 0}, {2, 2});

            Vec2 v = solver.FirstCellWithUndefWallsInPath();

            // need {0, 1}
            maze.Print();

            Serial.print(v.x);
            Serial.print(' ');
            Serial.println(v.y);
        }

        void CYCLOGRAMS(){
            cycloStore.addPrimitive(PrimitiveCycloAction_t::BLANK);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::STOP);

            cycloStore.addSmart(SmartCycloAction_t::IP90L, 2);
            cycloStore.addSmart(SmartCycloAction_t::FWD_X, 8);
            cycloStore.addSmart(SmartCycloAction_t::IP90R);
            cycloStore.addSmart(SmartCycloAction_t::IP180);
            cycloStore.addSmart(SmartCycloAction_t::DIAG_X, 30);
            cycloStore.addSmart(SmartCycloAction_t::FROM_ALIGN_TO_CENTER);
            cycloStore.addSmart(SmartCycloAction_t::SD135SR);

            cycloStore.printPrimitives();
            cycloStore.printSmarts();

            SmartSubmission ss = cycloStore.popFrontSmartSubmission();
            Serial.print((int)ss.smart);
            Serial.print(' ');
            Serial.println((int)ss.x);

            ss = cycloStore.popFrontSmartSubmission();
            Serial.print((int)ss.smart);
            Serial.print(' ');
            Serial.println((int)ss.x);

            cycloStore.reloadPrimitives();
            cycloStore.reloadSmarts();
        }

        void EXPLORER_LEFT_RIGHT_SMARTS(){
            cycloStore.addSmart(SmartCycloAction_t::SS90ER);
            cycloStore.addSmart(SmartCycloAction_t::SS90EL);
        }

        void FWD_3X()
        {
            cycloStore.addSmart(SmartCycloAction_t::FWD_X, 3);
        }

        void FWDE(){
            cycloStore.addSmart(SmartCycloAction_t::FWDE);
        }
        
        void EXPLORER_CYC()
        {
            cycloStore.addSmart(SmartCycloAction_t::FWD_X);
            cycloStore.addSmart(SmartCycloAction_t::SS90EL);
            cycloStore.addSmart(SmartCycloAction_t::SS90ER);
            cycloStore.addSmart(SmartCycloAction_t::IP180);

            cycloStore.printSmarts();

            cycloStore.reloadPrimitives();
            cycloStore.reloadSmarts();
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
                Direction __sd = Direction::S;
                actionsHandler.primitivesToExplorers(__sd);
            }

            maze.PrintDirPath();
            cycloStore.printPrimitives();
            cycloStore.printSmarts();
            cycloStore.reloadSmarts();
        }

        void OPTOCOUPLERS_SENSE(){
            optocoupler.printSense();
        }
        
        void OPTOCOUPLERS_MASK(){
            optocoupler.printMask();
        }

        void OPTOCOUPLERS_CELL(){
            optocoupler.printAbsCell();
        }

        void ANDREI_MOMENT(){
            // cycloStore.addSmart(SmartCycloAction_t::TO_ALIGN);
            // cycloStore.addSmart(SmartCycloAction_t::FROM_ALIGN_TO_CENTER);

            cycloStore.addSmart(SmartCycloAction_t::FWD_X, 1);
            cycloStore.addSmart(SmartCycloAction_t::SD135SL);
            cycloStore.addSmart(SmartCycloAction_t::DD90SR);
            cycloStore.addSmart(SmartCycloAction_t::DD90SL);
            cycloStore.addSmart(SmartCycloAction_t::DIAG_X, 1);
            cycloStore.addSmart(SmartCycloAction_t::DS135SL);
            cycloStore.addSmart(SmartCycloAction_t::FWD_X, 2);
            
            // cycloStore.addSmart(SmartCycloAction_t::DIAG_X, 1);
            // cycloStore.addSmart(SmartCycloAction_t::DD90SL);
            // cycloStore.addSmart(SmartCycloAction_t::DIAG_X, 3);
            // cycloStore.addSmart(SmartCycloAction_t::DS45SL);
        }

        void CONVERT_TO_SMART()
        {
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::STOP);
            cycloStore.printPrimitives();
            actionsHandler.convertToSmart();
            cycloStore.printSmarts();
        }
    }
}
