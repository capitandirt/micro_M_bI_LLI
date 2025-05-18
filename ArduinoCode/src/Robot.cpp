#include "Robot.h"

void Robot::init(){
    _maze->PrimaryFill();
    _maze->SetCell(START_CELL, START_ROBOT_COORDS);
}

void Robot::statusHandler(){
    switch (_statusSelector->getStatus())
    {
    case ProgramStatus::NONE:
    case ProgramStatus::NEED_START_COMMAND:
        _actionsHandler->inIdle();
        break;

    case ProgramStatus::PRE_ENTRY_START:
        _actionsHandler->needClusterDot();
        _statusSelector->nextStatus();
        break;

    case ProgramStatus::START_EXPLORER:
        start_explorer();
        break;
    
    case ProgramStatus::EXPLORER:
        step_flood_fill(FINISH_ROBOT_COORDS);
        break;

    case ProgramStatus::PRE_ENTRY_FINISH:
        _actionsHandler->needClusterDot();
        _statusSelector->nextStatus();
        break;

    case ProgramStatus::START_EXPLORER_AFTER_FINISH:
        start_explorer();
        break;

    case ProgramStatus::GO_TO_START:
        step_flood_fill(START_ROBOT_COORDS);
        break;

    default:
        break;
    }
}

void Robot::start_explorer(){
    if(!_cycloWorker->nowIsClusterDot()) return;
    _actionsHandler->reload();

    const WallState forward_wall = _optocoupler->getRelativeCell().north_wall;
    const Direction cur_dir = _odometry->getDir();
    
    if(forward_wall == WallState::HI){
        const Direction next_dir = _actionsHandler->needTurn(cur_dir);
        _odometry->updateDir(next_dir);
        return;
    }

    _statusSelector->nextStatus();
    _actionsHandler->needStartCellAligning();
    _cycloWorker->reload();
}

void Robot::step_flood_fill(const Vec2 end_cell)
{
    if(!_cycloWorker->nowIsClusterDot()) return;

    const Direction cur_robot_dir = _odometry->getDir();
    const Vec2 forward_robot_vec  = _odometry->getMazeCoords().plusOrtVector(cur_robot_dir);
    const Cell forward_cell       = _optocoupler->getCell(cur_robot_dir);

    if(forward_robot_vec.x == end_cell.x &&
       forward_robot_vec.y == end_cell.y 
    ){
        _odometry->updateMazeCoords(forward_robot_vec);
        _actionsHandler->needStop();
        _statusSelector->nextStatus();
        return;
    }

    _maze->SetCell(forward_cell, forward_robot_vec);
    _solver->SolveBfsMaze(forward_robot_vec, end_cell);

    _actionsHandler->loadExplorer(cur_robot_dir);    
    
    const Direction next_robot_dir = _maze->GetPathDir(0);

    _odometry->updateDir(next_robot_dir);
    _odometry->updateMazeCoords(forward_robot_vec);
}