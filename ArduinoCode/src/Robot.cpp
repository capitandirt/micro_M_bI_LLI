#include "Robot.h"

const PrimitiveCycloAction_t Robot::calcPrimitiveCycloAction(const uint8_t ind){
    if(ind >= _Maze->GetPathSize() - 1){
        return PrimitiveCycloAction_t::STOP;
    }
    
    return static_cast<PrimitiveCycloAction_t>(   
        (static_cast<int8_t>(_Maze->GetPathDir(ind)) - static_cast<int8_t>(_Maze->GetPathDir(ind + 1))
            + DIRECTION_SIZE) % DIRECTION_SIZE);
}

void Robot::DirsToPrimitives(PrimitiveCycloAction_t first_primitive){
    _cycloStore->reloadPrimitives();
    _cycloStore->addPrimitive(first_primitive);

    for(uint8_t i = 0; i < _Maze->GetPathSize() - 1; i++){
        _cycloStore->addPrimitive(calcPrimitiveCycloAction(i));
    }

    _cycloStore->printPrimitives();
}

void Robot::primitivesToExplorers()
{
    _cycloStore->reloadSmarts();
    _cycloStore->popFrontPrimitive();
    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);

    while(!_cycloStore->primitiveIsEmpty()){
        switch(_cycloStore->virtualPopFrontPrimitive()){
        case PrimitiveCycloAction_t::FORWARD:
            _cycloStore->addSmart(SmartCycloAction_t::FWD);
            _cycloStore->virtualPrimitiveRelease();
            break;

        case PrimitiveCycloAction_t::LEFT:
            _cycloStore->addSmart(SmartCycloAction_t::SS90EL);
            _cycloStore->virtualPrimitiveRelease();
            break;
        
        case PrimitiveCycloAction_t::RIGHT:
            _cycloStore->addSmart(SmartCycloAction_t::SS90ER);
            _cycloStore->virtualPrimitiveRelease();
            break;

        case PrimitiveCycloAction_t::STOP:
            _cycloStore->addSmart(SmartCycloAction_t::STOP);
            _cycloStore->virtualPrimitiveRelease();
            break;

        default:
            _cycloStore->addSmart(SmartCycloAction_t::IDLE);
            _cycloStore->virtualPrimitiveRelease();
        break;
        }
    }

    _cycloStore->printSmarts();
}

void Robot::primitivesToFasts(){
    
}

void Robot::pathToCyclogram(){
    // _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);  
    // for(uint8_t i = 0; i < _Maze->GetPathSize(); i++){
    //     // calcRelativeCycloAction(i);
    //     primitiveToFast(i);
    // }
    // Serial.println();
}

void Robot::moveFloodFill()
{
    Cell CellFromSensors = _optocoupler->getCellFromSensors(_odometry->getDir());
    _Maze->SetCell(CellFromSensors, _odometry->getMazeCoord());
    _solver->SolveBfsMaze(_odometry->getMazeCoord(), {MAZE_FINISH_CELLS_X, MAZE_FINISH_CELLS_Y});
    // primitivesToExplorer(FIRST);
}