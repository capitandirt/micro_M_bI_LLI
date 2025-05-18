#ifndef _MAZE_OBSERVER_H_
#define _MAZE_OBSERVER_H_

#include "Maze.h"
#include "Odometry.h"

enum class MazeCommand{
    NONE,
    ALIGN_IN_TURN,
    ALIGN_IN_IP180
};

struct MazeObserverConnectionParams{
    Maze* _maze;
    Odometry* _odometry;
};

class MazeObserver : public MazeObserverConnectionParams{
public:
    MazeObserver(MazeObserverConnectionParams* mocp):
        MazeObserverConnectionParams(*mocp){}
    
    
private:

private:
    MazeCommand _cur_maze_command = MazeCommand::NONE;
};

#endif // !_MAZE_OBSERVER_H_