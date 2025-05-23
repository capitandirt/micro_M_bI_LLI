#include "Robot.h"

void Robot::init(){
    _maze->PrimaryFill();
}

void Robot::statusHandler(){
    switch (_statusSelector->getStatus())
    {
    case ProgramStatus::NONE:
    case ProgramStatus::NEED_START_COMMAND:
        _actionsHandler->inIdle();
        break;

    case ProgramStatus::PRE_ENTRY_START:
        _actionsHandler->needDelay05();
        _statusSelector->nextStatus();
        break;

    case ProgramStatus::START_EXPLORER:
        start_explorer();
        break;
    
    case ProgramStatus::EXPLORER:
        step_flood_fill(FINISH_ROBOT_COORDS);
        break;

    case ProgramStatus::PRE_ENTRY_FINISH:
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

    const WallState forward_wall = _optocoupler->getRelativeCell().north_wall;
    const Direction cur_dir = _odometry->getDir();
    
    if(forward_wall == WallState::HI){
        const Direction next_dir = _actionsHandler->needTurn(cur_dir);
        
        const Vec2 cur_coords = _odometry->getMazeCoords();
        const Cell rel_cell = {forward_wall, WallState::LO, WallState::LO, WallState::LO};
        const Cell abs_cell = inDir(rel_cell, cur_dir);

        _maze->SetCell(abs_cell, cur_coords);
        _odometry->updateDir(next_dir);
        return;
    }

    _actionsHandler->needStartCellAligning();
    _statusSelector->nextStatus();
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
        _actionsHandler->needToEnd();
        _statusSelector->nextStatus();
        return;
    }

    _maze->SetCell(forward_cell, forward_robot_vec);
    _solver->SolveBfsMaze(forward_robot_vec, end_cell);

    _actionsHandler->exeExplorer(cur_robot_dir);    
    
    const Direction next_robot_dir = _maze->GetPathDir(0);

    _odometry->updateDir(next_robot_dir);
    _odometry->updateMazeCoords(forward_robot_vec);
}