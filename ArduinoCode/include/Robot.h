#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "CycloUtilits/ActionsHandler.h"
#include "Maze.h"
#include "Solver.h"
#include "OptocouplerSensors.h"
#include "Odometry.h"

struct RobotConnectionParams{
    ActionsHandler* _actionsHandler;
    Maze* _maze;
    Solver* _solver;
    OptocouplerSensors* _optocoupler;
    Odometry* _odometry;
};


class Robot : public RobotConnectionParams{
public:
    Robot(RobotConnectionParams* rcp): RobotConnectionParams(*rcp){}

    void stepFloodFill();
    bool checkFloodFill();
private:
    bool FLOOD_FILL_IS_FINISH = 0;

    Vec2 _buf_robot_coords;
    Direction _buf_robot_dir;
};

#endif // !_ROBOT_H_