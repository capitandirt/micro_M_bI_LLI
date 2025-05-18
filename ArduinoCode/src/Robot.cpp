#include "Robot.h"

void Robot::init(){
    _maze->PrimaryFill();
    _maze->SetCell(START_CELL, START_ROBOT_COORDS);
}

void Robot::startExplorer(){
    if(WAS_START_EXLORER || !(_cycloWorker->isCompleteCyclo() && _cycloWorker->nowIsClusterDot())) return;
    _actionsHandler->reload();

    const WallState forward_wall = _optocoupler->getRelativeCell().north_wall;
    const Direction cur_dir = _odometry->getDir();
    
    if(forward_wall == WallState::HI){
        const Direction next_dir = _actionsHandler->needTurn(cur_dir);
        _odometry->updateDir(next_dir);

        return;
    }

    WAS_START_EXLORER = 1;    
    _actionsHandler->needStartCellAligning();
    _cycloWorker->reload();
}

void Robot::stepFloodFill()
{
    if(!(_cycloWorker->isCompleteCyclo() && _cycloWorker->nowIsClusterDot()) || !WAS_START_EXLORER || FLOOD_FILL_IS_FINISH){
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