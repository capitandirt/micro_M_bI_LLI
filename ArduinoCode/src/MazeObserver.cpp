#include "MazeObserver.h"

MazeCommand MazeObserver::getCommand(const PrimitiveCycloAction_t primitive){
    const Vec2 cur_coords = _odometry->getMazeCoords();
    const Direction cur_dir = _odometry->getDir();

    const Cell_u cur_abs_cell = {.raw = _maze->GetCell(cur_coords)};

    const bool is_side_walls = toBool(cur_abs_cell.walls[toInt(incDir(cur_dir))]) &&
                               toBool(cur_abs_cell.walls[toInt(decDir(cur_dir))]); 

    const bool is_fwd_wall = toBool(cur_abs_cell.walls[toInt(cur_dir)]);
    

    if(primitive == PrimitiveCycloAction_t::BACK && is_fwd_wall){
        // _no_align_counter = 0;

        return MazeCommand::ALIGN_IN_IP180;
    }
    else return MazeCommand::NONE;

    // // if(primitive == PrimitiveCycloAction_t::FORWARD && is_side_walls){
    // //     _no_align_counter -= ALIGN_STEP;

    // //     if(_no_align_counter < 0)
    // //         _no_align_counter = 0;
    // // }
    // // else{
    //     _no_align_counter = (_no_align_counter + ALIGN_STEP);
        
    //     if(_no_align_counter > MAX_NO_ALIGN_COUNTER)
    //         _no_align_counter = MAX_NO_ALIGN_COUNTER;
    // // }

    // if(_no_align_counter < MAX_NO_ALIGN_COUNTER) return MazeCommand::NONE;
    
    // if((primitive == PrimitiveCycloAction_t::LEFT ||
    //     primitive == PrimitiveCycloAction_t::RIGHT) && is_fwd_wall){
    //     _no_align_counter = 0;
        
    //     return MazeCommand::ALIGN_IN_TURN;
    // }
    // else if(primitive == PrimitiveCycloAction_t::BACK && is_fwd_wall){
    //     _no_align_counter = 0;

    //     return MazeCommand::ALIGN_IN_IP180;
    // }
    // else return MazeCommand::NONE;
}