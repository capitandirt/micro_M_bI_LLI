#include "Robot.h"

const PrimitiveCycloAction_t Robot::calcPrimitiveCycloAction(const uint8_t ind){
    if(ind >= _maze->GetPathSize() - 1){
        return PrimitiveCycloAction_t::STOP;
    }
    auto dirNow  = static_cast<int8_t>(_maze->GetPathDir(ind));
    auto dirNext = static_cast<int8_t>(_maze->GetPathDir(ind + 1));
    return static_cast<PrimitiveCycloAction_t>((dirNow - dirNext + DIRECTION_SIZE) % DIRECTION_SIZE);
}

void Robot::DirsToPrimitives(){
    _cycloStore->reloadPrimitives();

    for(uint8_t i = 0; i < _maze->GetPathSize() - 1; i++){
        _cycloStore->addPrimitive(calcPrimitiveCycloAction(i));
    }

    _cycloStore->printPrimitives();
}

void Robot::start_primitive_process()
{
    const auto from_path_dir = static_cast<int8_t>(_maze->GetPathDir(0));
    const auto from_odom_dir = static_cast<int8_t>(_odometry->getDir());
    const auto first_primitive = static_cast<PrimitiveCycloAction_t>((from_path_dir - from_odom_dir + DIRECTION_SIZE) % DIRECTION_SIZE);

    switch(first_primitive) // установка соответствия направления робота и направлений пути
    {
        case PrimitiveCycloAction_t::LEFT:
            _cycloStore->addSmart(SmartCycloAction_t::IP90L);
            _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
            break;

        case PrimitiveCycloAction_t::RIGHT:
            _cycloStore->addSmart(SmartCycloAction_t::IP90R);
            _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
            break;

        case PrimitiveCycloAction_t::BACK:
            _cycloStore->addSmart(SmartCycloAction_t::IP180);
            _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);

            break;

        case PrimitiveCycloAction_t::FORWARD:
            _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
            break;

        default:
            break;
    }
}

void Robot::primitivesToExplorers()
{
    _cycloStore->reloadSmarts();
    start_primitive_process();

    while(!_cycloStore->primitiveIsEmpty()){
        switch(_cycloStore->popFrontPrimitive()){
        case PrimitiveCycloAction_t::FORWARD:
            _cycloStore->addSmart(SmartCycloAction_t::FWD);
            break;

        case PrimitiveCycloAction_t::LEFT:
            _cycloStore->addSmart(SmartCycloAction_t::SS90EL);
            break;
        
        case PrimitiveCycloAction_t::RIGHT:
            _cycloStore->addSmart(SmartCycloAction_t::SS90ER);
            break;

        case PrimitiveCycloAction_t::STOP:
            _cycloStore->addSmart(SmartCycloAction_t::STOP);
            break;

        default:
            _cycloStore->addSmart(SmartCycloAction_t::IDLE);
        break;
        }
    }

    _cycloStore->printSmarts();
}

void Robot::primitivesToFasts()
{
    _cycloStore->reloadSmarts();

    while(!_cycloStore->primitiveIsEmpty()){
        
    }

    _cycloStore->printSmarts();
}

void Robot::pathToCyclogram(){
    // _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);  
    // for(uint8_t i = 0; i < _maze->GetPathSize(); i++){
    //     // calcRelativeCycloAction(i);
    //     primitiveToFast(i);
    // }
    // Serial.println();
}

void Robot::moveFloodFill()
{
    Cell CellFromSensors = _optocoupler->getCellFromSensors(_odometry->getDir());
    _maze->SetCell(CellFromSensors, _odometry->getMazeCoord());
    _solver->SolveBfsMaze(_odometry->getMazeCoord(), {MAZE_FINISH_CELLS_X, MAZE_FINISH_CELLS_Y});
    // primitivesToExplorer(FIRST);
}