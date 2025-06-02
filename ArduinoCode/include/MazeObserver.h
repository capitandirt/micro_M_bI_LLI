#ifndef _MAZE_OBSERVER_H_
#define _MAZE_OBSERVER_H_

#include "Maze.h"
#include "Odometry.h"
#include "Drivers/OptocouplerSensors.h"
#include "CycloUtilits/CycloTypes.h"

enum class MazeCommand{
    NONE,
    START_ALIGN,
    ALIGN_IN_TURN,
    ALIGN_IN_IP180,
};

struct MazeObserverConnectionParams{
    Maze* const _maze;
    Odometry* const _odometry;
    OptocouplerSensors* const _optocouplers;
};

class MazeObserver : private MazeObserverConnectionParams{
public:
    MazeObserver(MazeObserverConnectionParams* mocp):
        MazeObserverConnectionParams(*mocp){}
    
    MazeCommand getCommand(const PrimitiveCycloAction_t primitive);
private:

private:
    static constexpr uint8_t ALIGN_STEP = 2;
    static constexpr int8_t MAX_NO_ALIGN_COUNTER = 4 * ALIGN_STEP;

    int16_t _no_align_counter = 0;

    MazeCommand _cur_maze_command = MazeCommand::NONE;
};

#endif // !_MAZE_OBSERVER_H_