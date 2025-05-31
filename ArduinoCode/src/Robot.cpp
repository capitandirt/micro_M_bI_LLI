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

    case ProgramStatus::DELAY_BEFORE_GO_FINISH:
        _actionsHandler->needDelay05();
        _statusSelector->nextStatus();
        break;

    case ProgramStatus::PRE_ENTRY_GO_FINISH:
        start_explorer();
        break;
    
    case ProgramStatus::GO_FINISH:
        step_flood_fill(FINISH_ROBOT_COORDS, TO_FINISH);
        break;

    // case ProgramStatus::DELAY_BEFORE_GO_START:
    //     _actionsHandler->needDelay05();
    //     _statusSelector->nextStatus();
    //     break;

    // case ProgramStatus::PRE_ENTRY_GO_START:
    //     start_explorer();
    //     break;

    // case ProgramStatus::GO_START:
    //     step_flood_fill(START_ROBOT_COORDS, TO_START);
    //     break;
        
    case ProgramStatus::PRE_ENTRY_FAST:
        start_fast();
        break;

    case ProgramStatus::FAST:

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

    Cell_u cur_cell;
    cur_cell.raw = _maze->GetCell(cur_coords);

    const Direction back_dir  = toOpposite(cur_dir);
    const WallState back_wall = cur_cell.walls[toInt(back_dir)];  

    if(toBool(back_wall)){  
        _actionsHandler->needStartCellAligning();
    }
    else{
        _actionsHandler->needFwdHalf();
    }

    _statusSelector->nextStatus();
}

void Robot::step_flood_fill(const Vec2 end_vec, const ExplorerStatus expl_status)
{
    if(!_cycloWorker->nowIsClusterDot()) return;

    const Direction cur_dir = _odometry->getDir();
    const Vec2 forward_vec  = _odometry->getMazeCoords().plusOrtVector(cur_dir);
    const Cell forward_cell = _optocoupler->getCell(cur_dir);

    _odometry->updateMazeCoords(forward_vec);

    if(try_end_to_finish(forward_vec, end_vec)) return;

    if(_maze->UndefWallInCell(forward_vec)){
        _maze->SetCell(forward_cell, forward_vec);
    }

    _solver->ExplorerSolveBfsMaze(forward_vec, end_vec);
    _actionsHandler->loadExplorer(cur_dir);    
    
    const Direction next_robot_dir = _maze->GetPathDir(0);
    _odometry->updateDir(next_robot_dir);
}

bool Robot::try_end_to_finish(const Vec2& cur, const Vec2& end){
    if(cur.x == end.x && cur.y == end.y){
        _actionsHandler->needFwdHalf();
        _statusSelector->nextStatus();
        return true;
    }

    return false;
}

void Robot::start_fast(){
    _odometry->updateDir(START_ROBOT_DIRECTION);

    _actionsHandler->primitivesToFasts();

    _statusSelector->nextStatus();
}