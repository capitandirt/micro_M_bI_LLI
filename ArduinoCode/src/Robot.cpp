#include "Robot.h"

void Robot::calcRelativeCycloAction(uint8_t ind){
    if(ind == _Maze->GetPathSize() - 1){
        _buf_relative_cyclo_action = PrimitiveCycloAction_t::STOP;
        return;
    }
    _buf_relative_cyclo_action = static_cast<PrimitiveCycloAction_t>(   
        (static_cast<int8_t>(_Maze->GetPathDir(ind)) - static_cast<int8_t>(_Maze->GetPathDir(ind + 1))
            + DIRECTION_SIZE) % DIRECTION_SIZE);
}

void Robot::convertPrimitiveToFastCyclogram(){
    switch (_buf_relative_cyclo_action)
    {
    case PrimitiveCycloAction_t::FORWARD:
        _cycloWorker->addAction(SmartCycloAction_t::FWD);
        break;

    case PrimitiveCycloAction_t::LEFT:
        _cycloWorker->addAction(SmartCycloAction_t::SS90SL);
        break;

    case PrimitiveCycloAction_t::RIGHT:
        _cycloWorker->addAction(SmartCycloAction_t::SS90SR);
        break;

    case PrimitiveCycloAction_t::STOP:
        _cycloWorker->addAction(SmartCycloAction_t::FWD_HALF);
        _cycloWorker->addAction(SmartCycloAction_t::STOP);
        break;

    case PrimitiveCycloAction_t::BLANK: 
        _cycloWorker->addAction(SmartCycloAction_t::IDLE);
        break;
    }
}

void Robot::convertPrimitiveToExplorerCyclogram()
{
    switch (_buf_relative_cyclo_action)
    {
    case PrimitiveCycloAction_t::FORWARD:
        _cycloWorker->addAction(SmartCycloAction_t::FWD);
        break;

    case PrimitiveCycloAction_t::LEFT:
        _cycloWorker->addAction(SmartCycloAction_t::SS90SL);
        break;

    case PrimitiveCycloAction_t::RIGHT:
        _cycloWorker->addAction(SmartCycloAction_t::SS90SR);
        break;

    case PrimitiveCycloAction_t::STOP:
        _cycloWorker->addAction(SmartCycloAction_t::FWD_HALF);
        _cycloWorker->addAction(SmartCycloAction_t::STOP);
        break;

    case PrimitiveCycloAction_t::BLANK: 
        _cycloWorker->addAction(SmartCycloAction_t::IDLE);
        break;
    }
}

void Robot::convertPathToCyclogram(){
    _cycloWorker->addAction(SmartCycloAction_t::FWD_HALF);
    for(uint8_t i = 0; i < _Maze->GetPathSize(); i++){
        calcRelativeCycloAction(i);
        convertPrimitiveToFastCyclogram();
    }
    // Serial.println();
}

void Robot::loadNextMoveFloodFill()
{
    Cell CellFromSensors = _optocoupler->getCellFromSensors(_odometry->getDir());
    _Maze->SetCell(CellFromSensors, _odometry->getMazeCoord());
    _solver->SolveBfsMaze(_odometry->getMazeCoord(), {MAZE_FINISH_CELLS_X, MAZE_FINISH_CELLS_Y});
    calcRelativeCycloAction(0);
    convertPrimitiveToExplorerCyclogram();
}