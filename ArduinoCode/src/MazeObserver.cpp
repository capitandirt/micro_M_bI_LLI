#include "MazeObserver.h"

MazeCommand MazeObserver::getCommand(PrimitiveCycloAction_t primitive){
    const bool forward_wall = toBool(_optocouplers->getRelativeCell().north_wall);
    const bool left_wall    = toBool(_optocouplers->getRelativeCell().west_wall);
    const bool right_wall   = toBool(_optocouplers->getRelativeCell().west_wall);
    
    _no_align_counter++;

    if(_no_align_counter < MAX_NO_ALIGN_COUNTER) return MazeCommand::NONE;

    if((primitive == PrimitiveCycloAction_t::LEFT || 
        primitive == PrimitiveCycloAction_t::RIGHT) &&
        forward_wall){
        
        _no_align_counter = 0;
        
        return MazeCommand::ALIGN_IN_TURN;
    }
    else if(primitive == PrimitiveCycloAction_t::BACK && forward_wall){
        _no_align_counter = 0;


             if(left_wall)  return MazeCommand::LEFT_ALIGN_IN_IP180;
        else if(right_wall) return MazeCommand::RIGHT_ALIGN_IN_IP180;
        else                return MazeCommand::FWD_ALIGN_IN_IP180;
    }
    else return MazeCommand::NONE;
}