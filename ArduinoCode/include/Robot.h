#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "CycloUtilits/ActionsHandler.h"
#include "Maze.h"
#include "Solver.h"
#include "Odometry.h"
#include "Drivers/StatusSelector.h"
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


class Robot : public RobotConnectionParams{
public:
    Robot(RobotConnectionParams* rcp): RobotConnectionParams(*rcp){}

    void init();

    void statusHandler();
    void startExplorer();

    void stepFloodFill(const Vec2 end_cell);

private:
    bool FLOOD_FILL_IS_FINISH = 0;
    bool WAS_START_EXLORER = 0;
};

#endif // !_ROBOT_H_