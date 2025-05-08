#include "Robot.h"

void Robot::init(){
    _maze->PrimaryFill();
    _maze->SetCell(START_CELL, START_ROBOT_COORDS);
    _actionsHandler->reload();
    _actionsHandler->needStartCellAligning();
}

void Robot::stepFloodFill()
{
    if(_odometry->getMazeCoords().x == FINISH_ROBOT_COORDS_X &&
       _odometry->getMazeCoords().y == FINISH_ROBOT_COORDS_Y 
    ){
        _actionsHandler->needStop();
        FLOOD_FILL_IS_FINISH = true;
        return;
    }

    const Direction cur_robot_dir = _odometry->getDir();

    const Vec2 cur_robot_vec = _odometry->getMazeCoords();
    const Vec2 next_robot_vec = _odometry->getMazeCoords().plusOrtVector(cur_robot_dir);

    const Cell cell_from_sensors = _optocoupler->getCell(cur_robot_dir);

    _maze->SetCell(cell_from_sensors, next_robot_vec);
    _solver->SolveBfsMaze(cur_robot_vec, FINISH_ROBOT_COORDS);

    _actionsHandler->reload();
    
    if(_optocoupler->cellIsImpasse()){
        _actionsHandler->needGetOutImpasse();
        _odometry->dirToOppositeSide();
    }
    else{
        const Direction next_robot_dir = _maze->GetPathDir(1);
        _odometry->updateDir(next_robot_dir);
    } 

    _actionsHandler->loadExplorer(cur_robot_dir);    
    _odometry->updateMazeCoords(cur_robot_dir);
}

bool Robot::checkFloodFill(){
    return FLOOD_FILL_IS_FINISH;
}
