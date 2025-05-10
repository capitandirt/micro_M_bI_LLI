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

    if(_odometry->getMazeCoords().x == FINISH_ROBOT_COORDS_X &&
       _odometry->getMazeCoords().y == FINISH_ROBOT_COORDS_Y 
    ){
        _actionsHandler->needStop();
        FLOOD_FILL_IS_FINISH = true;
        return;
    }

    Direction cur_robot_dir  = _odometry->getDir();

    const Vec2 cur_robot_vec = _odometry->getMazeCoords();
    const Vec2 forward_robot_vec = _odometry->getMazeCoords().plusOrtVector(cur_robot_dir);

    const Cell cell_from_sensors = _optocoupler->getCell(cur_robot_dir);

    Serial.println("NOW:");
    _maze->Print();
    _maze->PrintDirPath();
    Serial.print(_odometry->getMazeCoords().x);
    Serial.print(' ');
    Serial.println(_odometry->getMazeCoords().y);
    _odometry->printDir();

    _maze->SetCell(cell_from_sensors, forward_robot_vec);
    _solver->SolveBfsMaze(forward_robot_vec, FINISH_ROBOT_COORDS);

    _actionsHandler->reload();
    
    Direction next_robot_dir = _actionsHandler->loadExplorer(cur_robot_dir);    

    _odometry->updateDir(next_robot_dir);
    _odometry->updateMazeCoords(cur_robot_dir);

    Serial.println("BEFORE:");
    _maze->Print();
    _maze->PrintDirPath();
    Serial.print(_odometry->getMazeCoords().x);
    Serial.print(' ');
    Serial.println(_odometry->getMazeCoords().y);
    _odometry->printDir();
    Serial.println("=========================\n");


}

bool Robot::checkFloodFill(){
    return FLOOD_FILL_IS_FINISH;
}
