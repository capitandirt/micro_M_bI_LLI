#include "MazeObserver.h"

MazeCommand MazeObserver::getCommand(){
    const WallState forward_cell = _optocouplers->getRelativeCell().north_wall;

    if(forward_cell == WallState::HI){
        return MazeCommand::ALIGN_IN_TURN;
    }
    else return MazeCommand::NONE;
}