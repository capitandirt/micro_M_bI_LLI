#include "Robot.h"

void Robot::init(){
    _maze->PrimaryFill();
    _maze->SetCell(START_CELL, START_ROBOT_COORDS);
    
    _actionsHandler->reload();
    _actionsHandler->needStartCellAligning();

    _cycloWorker->init();
}

void Robot::stepFloodFill()
{
    if(!(_cycloWorker->isCompleteCyclo() && _cycloWorker->nowIsClusterDot()) || FLOOD_FILL_IS_FINISH){
        return;
    }

    const Direction cur_robot_dir = _odometry->getDir();
    const Vec2 forward_robot_vec  = _odometry->getMazeCoords().plusOrtVector(cur_robot_dir);
    const Cell forward_cell       = _optocoupler->getCell(cur_robot_dir);

    if(forward_robot_vec.x == FINISH_ROBOT_COORDS_X &&
       forward_robot_vec.y == FINISH_ROBOT_COORDS_Y 
    ){
        _actionsHandler->needStop();
        FLOOD_FILL_IS_FINISH = true;
        return;
    }

    _maze->SetCell(forward_cell, forward_robot_vec);
    _solver->SolveBfsMaze(forward_robot_vec, FINISH_ROBOT_COORDS);

    _actionsHandler->loadExplorer(cur_robot_dir);    
    
    const Direction next_robot_dir = _maze->GetPathDir(0);

    _odometry->updateDir(next_robot_dir);
    _odometry->updateMazeCoords(forward_robot_vec);
}