#include "Robot.h"
#include "Drivers/Led.h"
#include "Drivers/SlideCatcher.h"
#include "Drivers/ProgramStatusHandler.h"
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
extern ProgramStatusHandler programStatusSelector;
extern FunctionalSelector functionalSelector;
extern Robot robot;


ISR(TIMER2_COMPA_vect){
    battery.tick();
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
        programStatusSelector.init();
        optocoupler.init();

        robot.init();

        TIM2::INIT();
        
        gyro.init();

        float need_align = 0;
        if(analogRead(FUNCTION_PIN) > 700){
            need_align = 1;
        }
        indicator.on();
        while(need_align && millis() < 30000) 
        {
            // static uint16_t time0 = millis();
            // uint16_t time = millis();
            // static float theta_old = 0;
            gyro.tick();
            // float theta = gyro.getYawAngle();
            // Serial.println("init: " + String(theta));
            // if(abs(theta - theta_old) > 0.02)
            // {
            //     time0 = time;
            // }
            // if(time - time0 > 10000)
            // {
            //     need_calibration = false;
            // }
            // theta_old = theta;
        }
        indicator.off();
        gyro.setYaw0();

        odometry.reset();
    }

    void TICK(uint32_t now_millis){
        leftEncoder.tick();
        rightEncoder.tick();
        
        leftVelocityEstimator.tick();
        rightVelocityEstimator.tick();
        
        gyro.tick();
        
        leftServo.tick();
        rightServo.tick();

        optocoupler.calc();
        functionalSelector.tick();
        functionalSelector.decodeAdcReading();

        programStatusSelector.passMillis(now_millis);
        indicator.passMillis(now_millis);

        programStatusSelector.tick();

        slideCatcher.tick();
        odometry.tick(leftVelocityEstimator.getW(), rightVelocityEstimator.getW());
    }

    namespace TEST{
        void SET_SERIAL(){
            Serial.begin(115200);
        }

        void FUCKING_MAZE(){
            maze.Clear();

            maze.SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::HI}, {0, 0});
            maze.SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, {1, 0});
            maze.SetCell({WallState::HI, WallState::HI, WallState::LO, WallState::LO}, {2, 0});
            maze.SetCell({WallState::HI, WallState::LO, WallState::LO, WallState::HI}, {0, 1});
            maze.SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, {1, 1});
            maze.SetCell({WallState::LO, WallState::HI, WallState::HI, WallState::LO}, {2, 1});
            maze.SetCell({WallState::LO, WallState::LO, WallState::HI, WallState::HI}, {0, 2});
            maze.SetCell({WallState::HI, WallState::LO, WallState::HI, WallState::LO}, {1, 2});
            maze.SetCell({WallState::HI, WallState::HI, WallState::HI, WallState::LO}, {2, 2});

            maze.Print();
            solver.FastSolveBfsMaze({0, 0}, {2, 2});
            maze.Print();
            maze.PrintDirPath();
            //Serial.println(maze.GetPathSize());
        }

        void BFS(){
            solver.MazeTestConfig();
            
            solver.ExplorerSolveBfsMaze({0, 0}, {5, 5});
            
            maze.PrintDirPath();
            maze.Print();
        }
    
        void UNDEF_CELL_WALLS(){
            maze.PrimaryFill();

            maze.SetCell({WallState::HI, WallState::HI, WallState::UNDEF, WallState::UNDEF}, {1, 1});
            maze.SetCell({WallState::UNDEF, WallState::UNDEF, WallState::HI, WallState::UNDEF}, {2, 1});
            maze.SetCell({WallState::LO, WallState::LO, WallState::LO, WallState::LO}, {0, 0});

            solver.ExplorerSolveBfsMaze({0, 0}, {2, 2});

            Vec2 v = solver.firstUndefCellCoords();

            // need {0, 1}
            maze.Print();

            // Serial.print(v.x);
            // Serial.print(' ');
            // Serial.println(v.y);
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
            cycloStore.addSmart(SmartCycloAction_t::DIAG_X, 30);
            cycloStore.addSmart(SmartCycloAction_t::FROM_BACK_ALIGN_TO_CENTER);
            cycloStore.addSmart(SmartCycloAction_t::SD135SR);

            cycloStore.printPrimitives();
            cycloStore.printSmarts();

            SmartSubmission ss = cycloStore.popFrontSmartSubmission();
            // Serial.print((int)ss.smart);
            // Serial.print(' ');
            // Serial.println((int)ss.x);

            ss = cycloStore.popFrontSmartSubmission();
            // Serial.print((int)ss.smart);
            // Serial.print(' ');
            // Serial.println((int)ss.x);

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

            cycloStore.printSmarts();

            cycloStore.reloadPrimitives();
            cycloStore.reloadSmarts();
        }

        void CONVERT_PATH_TO_CYCLOGRAMS(){
            solver.MazeTestConfig();

            Vec2 __s = {0, 0}; Vec2 __f = {2, 2};    
            solver.ExplorerSolveBfsMaze(__s, __f);

            maze.Print();

            Direction __sd = Direction::S;
            // actionsHandler.primitivesToFasts(__sd);

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
            // cycloStore.addSmart(SmartCycloAction_t::TO_BACK_ALIGN);
            // cycloStore.addSmart(SmartCycloAction_t::FROM_BACK_ALIGN_TO_CENTER);

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

        void addTestMaze1()
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

            // SD45SL DS45SR SD180SL FWD_X1 SS90SL SD135SR DD90SL DD90SR DS135SL
        }

        void addTestMaze2()
        {
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);

            cycloStore.addPrimitive(PrimitiveCycloAction_t::STOP);

            // SS90SL SS90SR SS90SR SS90SL SD135SL DS45SR SD45SR DIAG_X1 DS45SR SD135SR DIAG_X1 DS34SR
        }

        void addTestMaze3()
        {
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);

            cycloStore.addPrimitive(PrimitiveCycloAction_t::STOP);

            // SS90SL SS90SR SD45SL DS45SR SD45SL DS45SR SD135SR DIAG_X1 DS45SR
        }

        void addTestMaze4()
        {
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);

            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);

            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            
            cycloStore.addPrimitive(PrimitiveCycloAction_t::STOP);

            //FWD_X1 SD45SL DIAG_X3 DS45SL SS90SL SD45SL DIAG_X3 DS45SL
        }
        void addTestMaze5()
        {
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);

            cycloStore.addPrimitive(PrimitiveCycloAction_t::STOP);

            //FWD_X6 SD135SL DIAG_X1 DS45SL
        }
        void addTestMaze6()
        {
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);

            cycloStore.addPrimitive(PrimitiveCycloAction_t::STOP);

            //FWD_X2 SS90SL SD45SR DS45SL FWD_X1 SS180SR FWD_X1
        }
        void addTestMaze7()
        {
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            
            
            cycloStore.addPrimitive(PrimitiveCycloAction_t::STOP);

            //FWD_X3 SD45SR DIAG_X2 DS45SL FWD_X2 SS180SR FWD_X1 SS90SL SS90SR
            
        }
        void addTestMaze8()
        {
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);

            cycloStore.addPrimitive(PrimitiveCycloAction_t::STOP);
            
            //FWD_X2 SD45SR DD90SL DD90SR DS45SL FWD_X2 SD45SR DIAG_X1 DS45SR SD45SR DIAG_X1 DS45SR FWD_X3
        }
        void addTestMaze9()
        {
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::STOP);

            //FWD_X4 SD45SR DIAG_X1 DS45SR FWD_X3
        }
        void addTestMaze10()
        {
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::STOP);

            //SD135SL DS135SR SS90SR FWD_X1
        }
        void addTestMaze12()
        {
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);

            cycloStore.addPrimitive(PrimitiveCycloAction_t::STOP);

            //FWD_X3 SS90SL FWD_X2 SS90SR SS180SL
        }
        void CONVERT_TO_SMART()
        {
            //======TEST MAZE 1======
            // addTestMaze1();
            //======================

            // cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            // cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            // cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            // cycloStore.addPrimitive(PrimitiveCycloAction_t::RIGHT);
            // cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            // cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            // cycloStore.addPrimitive(PrimitiveCycloAction_t::LEFT);
            // cycloStore.addPrimitive(PrimitiveCycloAction_t::FORWARD);
            // cycloStore.addPrimitive(PrimitiveCycloAction_t::STOP);
            
            addTestMaze12();
            
            //cycloStore.printPrimitives();
            converterToFasts.convert();
            // actionsHandler.primitivesToFasts();
            // cycloStore.printSmarts();
        }


        void PRIM_TO_FAST()
        {
            addTestMaze2();

            cycloStore.printPrimitives();
            cycloStore.printSmarts();

            actionsHandler.primitivesToFasts();
            
            cycloStore.printSmarts();
        }
    }
}
