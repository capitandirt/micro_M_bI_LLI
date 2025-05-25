#include "Robot.h"

void Robot::init(){
    _maze->PrimaryFill();
}

void Robot::statusHandler(){
    switch (_statusSelector->getStatus())
    {
    case ProgramStatus::NONE:
    case ProgramStatus::NEED_START_COMMAND:
        _actionsHandler->needIdle();
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
    const Vec2 cur_coords = _odometry->getMazeCoords();

    if(forward_wall == WallState::HI){
        const Direction next_dir = _actionsHandler->needTurn(cur_dir);
        
        const Cell rel_cell = {forward_wall, WallState::LO, WallState::LO, WallState::LO};
        const Cell abs_cell = inDir(rel_cell, cur_dir);

        _maze->SetCell(abs_cell, cur_coords);
        _odometry->updateDir(next_dir);
        return;
    }

    const Cell cur_cell = _maze->GetCell(cur_coords);
    const WallState back_wall = inDir(cur_cell, toOpposite(cur_dir)).south_wall;

    if(toBool(back_wall)){
        _actionsHandler->needStartCellAligning();
    }
    else{
        _actionsHandler->needClusterDot();
    }

    _statusSelector->nextStatus();
}

void Robot::step_flood_fill(const Vec2 end_vec)
{
    if(!_cycloWorker->nowIsClusterDot()) return;

    const Direction cur_dir = _odometry->getDir();
    const Vec2 forward_vec  = _odometry->getMazeCoords().plusOrtVector(cur_dir);
    const Cell forward_cell = _optocoupler->getCell(cur_dir);

    _odometry->updateMazeCoords(forward_vec);

    if(try_end(forward_vec, end_vec)) return;

    if(_maze->UndefWallInCell(forward_vec)){
        _maze->SetCell(forward_cell, forward_vec);
    }

    _solver->SolveBfsMaze(forward_vec, end_vec);
    _actionsHandler->loadExplorer(cur_dir);    
    
    const Direction next_robot_dir = _maze->GetPathDir(0);
    _odometry->updateDir(next_robot_dir);
}

bool Robot::try_end(const Vec2 cur, const Vec2 end){
    if(cur.x == end.x &&
       cur.y == end.y 
    ){
        const Direction cur_dir = _odometry->getDir();
        const Direction next_dir = _actionsHandler->needToEnd(cur_dir);

        _statusSelector->nextStatus();
        return true;
    }

    return false;
}