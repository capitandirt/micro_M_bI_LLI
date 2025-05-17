#include "Robot.h"

void Robot::init(){
    _maze->PrimaryFill();
    _maze->SetCell(START_CELL, START_ROBOT_COORDS);
}

void Robot::statusHandler(){
    switch (_functionalSelector->getStatus())
    {
    case ProgramStatus::PRE_ENTRY_START:
        _actionsHandler->needClusterDot();
        _functionalSelector->incStatus();
        break;

    case ProgramStatus::START_EXPLORER:
        startExplorer();
        break;
    
    case ProgramStatus::EXPLORER:
        stepFloodFill(FINISH_ROBOT_COORDS);
        break;

    case ProgramStatus::PRE_ENTRY_FINISH:
        _actionsHandler->needClusterDot();
        _functionalSelector->incStatus();
        break;

    case ProgramStatus::START_EXPLORER_AFTER_FINISH:
        startExplorer();
        break;

    case ProgramStatus::GO_TO_START:
        stepFloodFill(START_ROBOT_COORDS);
        break;

    default:
        break;
    }

}

void Robot::startExplorer(){
    if(!_cycloWorker->nowIsClusterDot()) return;
    _actionsHandler->reload();

    const WallState forward_wall = _optocoupler->getRelativeCell().north_wall;
    const Direction cur_dir = _odometry->getDir();
    
    if(forward_wall == WallState::HI){
        const Direction next_dir = _actionsHandler->needTurn(cur_dir);
        _odometry->updateDir(next_dir);
        return;
    }

    _functionalSelector->incStatus();
    _actionsHandler->needStartCellAligning();
    _cycloWorker->reload();
}

void Robot::stepFloodFill(const Vec2 end_cell)
{
    if(!_cycloWorker->nowIsClusterDot()) return;

    const Direction cur_robot_dir = _odometry->getDir();
    const Vec2 forward_robot_vec  = _odometry->getMazeCoords().plusOrtVector(cur_robot_dir);
    const Cell forward_cell       = _optocoupler->getCell(cur_robot_dir);

    if(forward_robot_vec.x == end_cell.x &&
       forward_robot_vec.y == end_cell.y 
    ){
        _actionsHandler->needStop();
        _functionalSelector->incStatus();
        return;
    }

    _maze->SetCell(forward_cell, forward_robot_vec);
    _solver->SolveBfsMaze(forward_robot_vec, end_cell);

    _actionsHandler->loadExplorer(cur_robot_dir);    
    
    const Direction next_robot_dir = _maze->GetPathDir(0);

    _odometry->updateDir(next_robot_dir);
    _odometry->updateMazeCoords(forward_robot_vec);
}