#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "CycloUtilits/ActionsHandler.h"
#include "Maze.h"
#include "Solver.h"
#include "Odometry.h"
#include "Drivers/SlideCatcher.h"
#include "Drivers/OptocouplerSensors.h"
#include "Drivers/ProgramStatusHandler.h"
#include "Drivers/FunctionalSelector.h"

struct RobotConnectionParams{
    CycloWorker* _cycloWorker;
    ActionsHandler* _actionsHandler;
    Maze* _maze;
    Solver* _solver;
    OptocouplerSensors* _optocoupler;
    Odometry* _odometry;
    ProgramStatusHandler* _programStatusSelector;
    FunctionalSelector* _functionalSelector;
    SlideCatcher* _slideCatcher;
};


class Robot : private RobotConnectionParams{
public:
    Robot(RobotConnectionParams* rcp): RobotConnectionParams(*rcp){}

    void init();
    void stateMachine();

private: 
    enum ExplorerStatus{
        TO_START,
        TO_FINISH
    };

    enum Lever_type{
        LEVER_FIRST_DIRECTION = 0, 
        LEVER_EXPLORER_WITH_HIGH_SPEED = 1,
        LEVER_NOT_GO_START = 2,
        LEVER_RACE_TYPE = 3
    };

    void start_explorer(const ExplorerStatus expl_status);
    void step_flood_fill(const Vec2 end_cell, ExplorerStatus explorer_status);

    bool try_end(const Vec2& cur, const Vec2& end, ExplorerStatus explorer_status);

    void convert_to_fast();
    void fast();
};

#endif // !_ROBOT_H_