#include "Robot.h"

void Robot::init(){
    _maze->PrimaryFill();
}

void Robot::stateMachine(){
    switch (_programStatusSelector->getStatus())
    {
    case ProgramStatus::NEED_START_PROGRAM_COMMAND:
        _actionsHandler->needIdle();
        break;

    case ProgramStatus::ESTIMATE_FAST_OR_EXPLORER:
        if(_functionalSelector->isLever(3))
            _programStatusSelector->setStatus(ProgramStatus::NEED_FAST_SLIDE);
        else 
            _programStatusSelector->setStatus(ProgramStatus::NEED_EXPLORER_SLIDE);
        break;

    case ProgramStatus::NEED_EXPLORER_SLIDE:
        break;

    case ProgramStatus::DELAY_BEFORE_GO_FINISH:
        explorer_status = TO_FINISH;

        if(_functionalSelector->isLever(0)){
            _odometry->setDir(Direction::S);
        }
        else _odometry->setDir(Direction::E);

        _odometry->updateMazeCoords(START_ROBOT_COORDS);
        
        _actionsHandler->needDelay05();
        _programStatusSelector->nextStatus();
        break;

    case ProgramStatus::PRE_ENTRY_GO_FINISH:
        start_explorer(TO_FINISH);
        break;
    
    case ProgramStatus::GO_FINISH:
        step_flood_fill(FINISH_ROBOT_COORDS);
        break;

    case ProgramStatus::DELAY_BEFORE_GO_START:
        explorer_status = TO_START;

        _actionsHandler->needDelay05();
        _programStatusSelector->nextStatus();
        break;

    case ProgramStatus::PRE_ENTRY_GO_START:
        start_explorer(TO_START);
        break;

    case ProgramStatus::GO_START:
        step_flood_fill(START_ROBOT_COORDS);
        break;

    case ProgramStatus::NEED_FAST_SLIDE:
        break;

    case ProgramStatus::DELAY_BEFORE_FAST:
        _actionsHandler->needDelay05();
        _programStatusSelector->nextStatus();
        break;

    case ProgramStatus::PRE_ENTRY_FAST:
        statusConvertToSmart();
        _programStatusSelector->nextStatus();
        break;    

    case ProgramStatus::FAST:
        break;

    default:
        break;
    }
}

void Robot::start_explorer(const ExplorerStatus expl_status){
    if(!_cycloWorker->nowIsClusterDot()) return;

    const Vec2 cur_coords = _odometry->getMazeCoords();
    const Direction cur_dir = _odometry->getDir();
    
    const Cell_u cur_cell{.raw = _maze->GetCell(cur_coords)};
    const WallState forward_wall = cur_cell.walls[toInt(cur_dir)];

    if(toBool(forward_wall)){
        const Direction next_dir = _actionsHandler->needTurn(cur_dir);
        _odometry->setDir(next_dir);
        return;
    }

    const Direction back_dir  = toOpposite(cur_dir);
    const WallState back_wall = cur_cell.walls[toInt(back_dir)];  

    if(toBool(back_wall)){  
        _actionsHandler->needStartCellAligning();
    }
    else{
        _actionsHandler->needFwdHalf();
    }

    _programStatusSelector->nextStatus();
}

void Robot::step_flood_fill(const Vec2 end_vec)
{
    if(!_cycloWorker->nowIsClusterDot()) return;

    const Direction cur_dir = _odometry->getDir();
    const Vec2 forward_vec  = _odometry->getMazeCoords().plusOrtVector(cur_dir);
    const Cell forward_cell = _optocoupler->getCell(cur_dir);

    _odometry->updateMazeCoords(forward_vec);

    if(_maze->UndefWallInCell(forward_vec)){
        _maze->SetCell(forward_cell, forward_vec);
    }

    if(try_end(forward_vec, end_vec)) return;

    _solver->ExplorerSolveBfsMaze(forward_vec, end_vec);
    _actionsHandler->loadExplorer(cur_dir, _functionalSelector->isLever(1));    
    
    const Direction next_robot_dir = _maze->GetPathDir(0);
    _odometry->setDir(next_robot_dir);
}

bool Robot::try_end(const Vec2& cur, const Vec2& end){
    if(cur.x == end.x && cur.y == end.y){
        _actionsHandler->needFwdHalf();
        
        if(explorer_status == TO_FINISH){
            if(_functionalSelector->isLever(2)){
                _programStatusSelector->setStatus(ProgramStatus::NEED_START_PROGRAM_COMMAND);
            }
            else _programStatusSelector->nextStatus();
        }
        else{
            _programStatusSelector->setStatus(ProgramStatus::NEED_START_PROGRAM_COMMAND);
        }

        return true;
    }
    return false;
}

void Robot::statusConvertToSmart()
{
    _solver->FastSolveBfsMaze(START_ROBOT_COORDS, FINISH_ROBOT_COORDS);
    _actionsHandler->loadFasts();
}


void Robot::fast(){
    
}