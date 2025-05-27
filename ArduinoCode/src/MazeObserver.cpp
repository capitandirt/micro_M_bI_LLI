#include "MazeObserver.h"

MazeCommand MazeObserver::getCommand(PrimitiveCycloAction_t primitive){
    const bool is_back_walls = toBool(_optocouplers->getRelativeCell().east_wall) &&
                               toBool(_optocouplers->getRelativeCell().west_wall); 

    const bool forward_wall = toBool(_optocouplers->getRelativeCell().north_wall);
    
    if(primitive == PrimitiveCycloAction_t::FORWARD && is_back_walls && _no_align_counter >= 1){
        _no_align_counter = 0;
    }
    else _no_align_counter++;

    if(_no_align_counter < MAX_NO_ALIGN_COUNTER) return MazeCommand::NONE;
    
    if((primitive == PrimitiveCycloAction_t::LEFT ||
        primitive == PrimitiveCycloAction_t::RIGHT) &&
        forward_wall){
        
        _no_align_counter = 0;
        
        return MazeCommand::ALIGN_IN_TURN;
    }
    else if(primitive == PrimitiveCycloAction_t::BACK && forward_wall){
        
        _no_align_counter = 0;

        return MazeCommand::ALIGN_IN_IP180;
    }
    else return MazeCommand::NONE;
}