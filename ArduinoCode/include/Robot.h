#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "CycloUtilits/ActionsHandler.h"
#include "Maze.h"
#include "Solver.h"
#include "Drivers/OptocouplerSensors.h"
#include "Odometry.h"

struct RobotConnectionParams{
    CycloWorker* _cycloWorker;
    ActionsHandler* _actionsHandler;
    Maze* _maze;
    Solver* _solver;
    OptocouplerSensors* _optocoupler;
    Odometry* _odometry;
};


class Robot : public RobotConnectionParams{
public:
    Robot(RobotConnectionParams* rcp): RobotConnectionParams(*rcp){}

    void init();

    void startExplorer();

    void stepFloodFill();

private:
    bool FLOOD_FILL_IS_FINISH = 0;
    bool WAS_START_EXLORER = 0;
};

#endif // !_ROBOT_H_