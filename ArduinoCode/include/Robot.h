#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "CycloWorker.h"
#include "Solver.h"
#include "Maze.h"
#include "OptocouplerSensors.h"

struct RobotConnectionParams{
    CycloWorker* _cycloWorker;
    Solver* _solver;
    Maze* _Maze;
    OptocouplerSensors* _optocoupler;
    Odometry* _odometry;
};

class Robot : public RobotConnectionParams{
public:
    Robot(RobotConnectionParams* rcp): RobotConnectionParams(*rcp){}

    const PrimitiveCycloAction_t calcRelativeCycloAction(const uint8_t ind);
    
    void convertPrimitiveToFastCyclogram(uint8_t& ind);
    void convertPrimitiveToExplorerCyclogram(uint8_t& ind);
    void convertPathToCyclogram();

    void loadNextMoveFloodFill();
private:
    PrimitiveCycloAction_t _buf_relative_cyclo_action;
};

#endif // !_ROBOT_H_