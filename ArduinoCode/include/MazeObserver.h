#ifndef _MAZE_OBSERVER_H_
#define _MAZE_OBSERVER_H_

#include "Maze.h"
#include "Odometry.h"
#include "Drivers/OptocouplerSensors.h"
#include "CycloUtilits/CycloTypes.h"

enum class MazeCommand{
    NONE,
    ALIGN_IN_TURN,
    FWD_ALIGN_IN_IP180,
    LEFT_ALIGN_IN_IP180,
    RIGHT_ALIGN_IN_IP180,
};

struct MazeObserverConnectionParams{
    // Maze* _maze;
    // Odometry* _odometry;
    OptocouplerSensors* _optocouplers;
};

class MazeObserver : public MazeObserverConnectionParams{
public:
    MazeObserver(MazeObserverConnectionParams* mocp):
        MazeObserverConnectionParams(*mocp){}
    
    MazeCommand getCommand(PrimitiveCycloAction_t primitive);
private:

private:
    static constexpr int8_t MAX_NO_ALIGN_COUNTER = 3;

    uint8_t _no_align_counter = 0;

    MazeCommand _cur_maze_command = MazeCommand::NONE;
};

#endif // !_MAZE_OBSERVER_H_