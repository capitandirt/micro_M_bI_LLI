#include "Robot.h"

void Robot::stepFloodFill()
{
    Cell cell_from_sensors = _optocoupler->getCell(_odometry->getDir());
    
    _maze->SetCell(cell_from_sensors, _odometry->getMazeCoords());
    _solver->SolveBfsMaze(_odometry->getMazeCoords(), {MAZE_FINISH_CELLS_X, MAZE_FINISH_CELLS_Y});
    
    _actionsHandler->primitivesToExplorers(_odometry->getDir(), _odometry->getMazeCoords(), _buf_robot_coords);
    
    _odometry->updateDir(_buf_robot_dir);
    _odometry->updateMazeCoords(_buf_robot_coords);
}