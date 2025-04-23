#include "Robot.h"

const PrimitiveCycloAction_t Robot::calcRelativeCycloAction(const uint8_t ind){
    if(ind >= _Maze->GetPathSize() - 1){
        return PrimitiveCycloAction_t::STOP;
    }
    return static_cast<PrimitiveCycloAction_t>(   
        (static_cast<int8_t>(_Maze->GetPathDir(ind)) - static_cast<int8_t>(_Maze->GetPathDir(ind + 1))
            + DIRECTION_SIZE) % DIRECTION_SIZE);
}

void Robot::convertPrimitiveToFastCyclogram(uint8_t& ind){
    switch (calcRelativeCycloAction(ind))
    {
    case PrimitiveCycloAction_t::FORWARD:
        switch(calcRelativeCycloAction(++ind)){
        case PrimitiveCycloAction_t::RIGHT:
            switch (calcRelativeCycloAction(++ind))
            {
            case PrimitiveCycloAction_t::LEFT:
                switch (calcRelativeCycloAction(++ind))
                {
                case PrimitiveCycloAction_t::FORWARD:
                    _cycloWorker->addAction(SmartCycloAction_t::FWD_HALF);
                    _cycloWorker->addAction(SmartCycloAction_t::DS45SR);
                    _cycloWorker->addAction(SmartCycloAction_t::DS45SL);
                    _cycloWorker->addAction(SmartCycloAction_t::FWD);
                    break;
                
                default:
                    ind -= 3;
                    _cycloWorker->addAction(SmartCycloAction_t::FWD);
                }
                break;
            default:
                ind -= 2;
                _cycloWorker->addAction(SmartCycloAction_t::FWD);
            }
            break;
        case PrimitiveCycloAction_t::LEFT:
            switch (calcRelativeCycloAction(++ind))
            {
            case PrimitiveCycloAction_t::RIGHT:
                switch (calcRelativeCycloAction(++ind))
                {
                case PrimitiveCycloAction_t::FORWARD:
                    _cycloWorker->addAction(SmartCycloAction_t::FWD_HALF);
                    _cycloWorker->addAction(SmartCycloAction_t::DS45SL);
                    _cycloWorker->addAction(SmartCycloAction_t::DS45SR);
                    _cycloWorker->addAction(SmartCycloAction_t::FWD);
                    break;
                
                default:
                    ind -= 3;
                    _cycloWorker->addAction(SmartCycloAction_t::FWD);
                }
                break;
            default:
                ind -= 2;
                _cycloWorker->addAction(SmartCycloAction_t::FWD);
            }
            break;
        default:
            ind -= 1;
            _cycloWorker->addAction(SmartCycloAction_t::FWD);
        }
        break;

    case PrimitiveCycloAction_t::LEFT:
        switch(calcRelativeCycloAction(++ind)){
        case PrimitiveCycloAction_t::LEFT:
            _cycloWorker->addAction(SmartCycloAction_t::SS180L);
            break;
        default:
            --ind;
            _cycloWorker->addAction(SmartCycloAction_t::SS90SL);
        }
        break;

    case PrimitiveCycloAction_t::RIGHT:
        switch(calcRelativeCycloAction(++ind)){
        case PrimitiveCycloAction_t::RIGHT:
            _cycloWorker->addAction(SmartCycloAction_t::SS180R);
            break;
        default:
            --ind;
            _cycloWorker->addAction(SmartCycloAction_t::SS90SR);
        }
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

void Robot::convertPrimitiveToExplorerCyclogram(uint8_t& ind)
{
    switch (calcRelativeCycloAction(ind))
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
        // calcRelativeCycloAction(i);
        convertPrimitiveToFastCyclogram(i);
    }
    // Serial.println();
}

void Robot::loadNextMoveFloodFill()
{
    Cell CellFromSensors = _optocoupler->getCellFromSensors(_odometry->getDir());
    _Maze->SetCell(CellFromSensors, _odometry->getMazeCoord());
    _solver->SolveBfsMaze(_odometry->getMazeCoord(), {MAZE_FINISH_CELLS_X, MAZE_FINISH_CELLS_Y});
    //convertPrimitiveToExplorerCyclogram(0);
}