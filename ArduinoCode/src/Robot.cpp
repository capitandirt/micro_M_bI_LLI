#include "Robot.h"

void Robot::moveFloodFill()
{
    Cell CellFromSensors = _optocoupler->getCellFromSensors(_odometry->getDir());
    _maze->SetCell(CellFromSensors, _odometry->getMazeCoord());
    _solver->SolveBfsMaze(_odometry->getMazeCoord(), {MAZE_FINISH_CELLS_X, MAZE_FINISH_CELLS_Y});
    _actionsHandler->primitivesToExplorers();
}