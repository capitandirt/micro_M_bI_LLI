#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "CycloUtilits/ActionsHandler.h"
#include "Maze.h"
#include "Solver.h"
#include "Odometry.h"
#include "Drivers/StatusSelector.h"
#include "Drivers/SlideCatcher.h"
#include "Drivers/OptocouplerSensors.h"

struct RobotConnectionParams{
    CycloWorker* _cycloWorker;
    ActionsHandler* _actionsHandler;
    Maze* _maze;
    Solver* _solver;
    OptocouplerSensors* _optocoupler;
    Odometry* _odometry;
    StatusSelector* _statusSelector;
};


class Robot : private RobotConnectionParams{
public:
    Robot(RobotConnectionParams* rcp): RobotConnectionParams(*rcp){}

    void init();
    void statusHandler();

private:
    enum ExplorerStatus{
        TO_START,
        TO_FINISH
    };

    void start_explorer();
    void step_flood_fill(const Vec2 end_cell, const ExplorerStatus expl_status);

    bool try_end_to_finish(const Vec2& cur, const Vec2& end);
    bool try_end_to_start(const Vec2& cur, const Vec2& end);

    void fast();
};

#endif // !_ROBOT_H_