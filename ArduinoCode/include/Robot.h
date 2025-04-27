#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "CycloUtilits/CycloStore.h"
#include "Solver.h"
#include "Maze.h"
#include "OptocouplerSensors.h"

struct RobotConnectionParams{
    CycloStore* _cycloStore;
    Solver* _solver;
    Maze* _maze;
    OptocouplerSensors* _optocoupler;
    Odometry* _odometry;
};

class Robot : public RobotConnectionParams{
public:
    Robot(RobotConnectionParams* rcp): RobotConnectionParams(*rcp){}

    const PrimitiveCycloAction_t calcPrimitiveCycloAction(const uint8_t ind);
    

    void DirsToPrimitives();
    void primitivesToExplorers();
    void primitivesToFasts();
    
    void pathToCyclogram();
    
    void moveFloodFill();

private:
    void start_primitive_process();

private:
    static constexpr uint8_t FIRST = 0;

    PrimitiveCycloAction_t _buf_relative_cyclo_action;
};

#endif // !_ROBOT_H_