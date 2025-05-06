#include "Robot.h"

void Robot::stepFloodFill()
{
    if(_odometry->getMazeCoords().x == FINISH_ROBOT_COORDS_X &&
       _odometry->getMazeCoords().y == FINISH_ROBOT_COORDS_Y ){
        _actionsHandler->needStop();
        FLOOD_FILL_IS_FINISH = true;
        return;
    }

    Cell cell_from_sensors = _optocoupler->getCell(_odometry->getDir());
    _maze->SetCell(cell_from_sensors, _odometry->getMazeCoords().plusOrtVector(_odometry->getDir()));

    _solver->SolveBfsMaze(_odometry->getMazeCoords(), FINISH_ROBOT_COORDS);
    
    _maze->Print();


    _actionsHandler->loadExplorer(_odometry->getDir());
    
    _buf_robot_dir = _maze->GetPathDir(1);
    
    _odometry->updateMazeCoords(_odometry->getDir());
    _odometry->updateDir(_buf_robot_dir);
}

bool Robot::checkFloodFill(){
    return FLOOD_FILL_IS_FINISH;
}
