#include "Robot.h"

void Robot::stepFloodFill()
{
    Cell cell_from_sensors = _optocoupler->getCell(_odometry->getDir());
    
    _maze->SetCell(cell_from_sensors, _odometry->getMazeCoords());
    _solver->SolveBfsMaze(_odometry->getMazeCoords(), FINISH_ROBOT_COORDS);
    
    _maze->Print();
    _actionsHandler->loadExplorer(_odometry->getDir());
    
    _buf_robot_dir = _maze->GetPathDir(0);

    _odometry->updateDir(_buf_robot_dir);
    _odometry->updateMazeCoords(_buf_robot_dir); // изменяем координаты в лабиринте по диру
}