#include "Robot.h"
#include "Drivers/Led.h"
#include "Drivers/SlideCatcher.h"
#include "Drivers/StatusSelector.h"
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
extern Led indicator;
extern SlideCatcher slideCatcher;
extern StatusSelector statusSelector;
extern Robot robot;


ISR(TIMER2_COMPA_vect){
    indicator.tick();
    optocoupler.tick();
}

namespace DEVICES{  
    void INIT(){
        leftEncoder.init();
        rightEncoder.init();

        leftMotor.init();
        rightMotor.init();
        
        indicator.init();
        statusSelector.init();
        optocoupler.init();
        
        // robot.init();

        TIM2::INIT();
    }

    void TICK(uint32_t now_millis){
        leftEncoder.tick();
        rightEncoder.tick();
        
        leftVelocityEstimator.tick();
        rightVelocityEstimator.tick();
        
        leftServo.tick();
        rightServo.tick();
        
        statusSelector.passMillis(now_millis);
        indicator.passMillis(now_millis);

        statusSelector.tick();

        if(statusSelector.getStatus() == ProgramStatus::NEED_START_COMMAND){
            slideCatcher.tick();
        }

        odometry.tick(leftVelocityEstimator.getW(), rightVelocityEstimator.getW());
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

            Vec2 __s = {0, 0}; Vec2 __f = {2, 2};    
            solver.SolveBfsMaze(__s, __f);

            maze.Print();

            Direction __sd = Direction::S;
            actionsHandler.primitivesToExplorers(__sd);

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
            cycloStore.addSmart(SmartCycloAction_t::TO_ALIGN);
            cycloStore.addSmart(SmartCycloAction_t::FROM_ALIGN_TO_CENTER);
            cycloStore.addSmart(SmartCycloAction_t::SD45SL);
            cycloStore.addSmart(SmartCycloAction_t::DS45SR);
            cycloStore.addSmart(SmartCycloAction_t::SS180SL);
            cycloStore.addSmart(SmartCycloAction_t::FWD_X);
            cycloStore.addSmart(SmartCycloAction_t::SS90SR);
            cycloStore.addSmart(SmartCycloAction_t::SD135SR);
            cycloStore.addSmart(SmartCycloAction_t::DD90SL);
            cycloStore.addSmart(SmartCycloAction_t::DD90SR);
            cycloStore.addSmart(SmartCycloAction_t::DS135SL);
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
