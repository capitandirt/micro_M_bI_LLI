#include "Robot.h"

const PrimitiveCycloAction_t Robot::calcPrimitiveCycloAction(const uint8_t ind){
    if(ind >= _Maze->GetPathSize() - 1){
        return PrimitiveCycloAction_t::STOP;
    }
    return static_cast<PrimitiveCycloAction_t>(   
        (static_cast<int8_t>(_Maze->GetPathDir(ind)) - static_cast<int8_t>(_Maze->GetPathDir(ind + 1))
            + DIRECTION_SIZE) % DIRECTION_SIZE);
}

void Robot::primitiveToFast(uint8_t& ind){
    const uint8_t ind_before = ind;

    switch (calcPrimitiveCycloAction(ind))
    {
    case PrimitiveCycloAction_t::FORWARD:
        switch (calcPrimitiveCycloAction(++ind))
        {
        case PrimitiveCycloAction_t::LEFT:
            switch (calcPrimitiveCycloAction(++ind))
            {
            case PrimitiveCycloAction_t::LEFT:
                switch(calcPrimitiveCycloAction(++ind))
                {
                case PrimitiveCycloAction_t::RIGHT:
                    switch(calcPrimitiveCycloAction(++ind)){
                    case PrimitiveCycloAction_t::FORWARD:
                        _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
                        _cycloStore->addSmart(SmartCycloAction_t::SD135SR);
                        _cycloStore->addSmart(SmartCycloAction_t::DS45SL);
                        _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
                        break;
                    default:
                        ind = ind_before;
                    }
                    break;
                default:
                    ind = ind_before;
                }
                break;

            case PrimitiveCycloAction_t::RIGHT:
                switch (calcPrimitiveCycloAction(++ind))
                {
                case PrimitiveCycloAction_t::FORWARD:
                    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
                    _cycloStore->addSmart(SmartCycloAction_t::DS45SL);
                    _cycloStore->addSmart(SmartCycloAction_t::DS45SR);
                    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
                    break;
                default:
                    ind = ind_before;
                }
                break;
            default:
                ind = ind_before;
            }  
            break;

        case PrimitiveCycloAction_t::RIGHT:
            switch (calcPrimitiveCycloAction(++ind))
            {
            case PrimitiveCycloAction_t::LEFT:
                switch (calcPrimitiveCycloAction(++ind))
                {
                case PrimitiveCycloAction_t::FORWARD:
                    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
                    _cycloStore->addSmart(SmartCycloAction_t::DS45SR);
                    _cycloStore->addSmart(SmartCycloAction_t::DS45SL);
                    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
                    break;
                default:
                    ind = ind_before;
                }
                break;

            case PrimitiveCycloAction_t::RIGHT:
                switch(calcPrimitiveCycloAction(++ind)){
                case PrimitiveCycloAction_t::LEFT:
                    switch(calcPrimitiveCycloAction(++ind)){
                    case PrimitiveCycloAction_t::FORWARD:
                        _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
                        _cycloStore->addSmart(SmartCycloAction_t::SD135SR);
                        _cycloStore->addSmart(SmartCycloAction_t::DS45SL);
                        _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);

                        break;
                    default:
                        ind = ind_before;                        
                    }
                    break;
                default:
                    ind = ind_before;
                }
                break;

            default:
                ind = ind_before;
            }  
        default:
            ind = ind_before;
        }

        if(ind == ind_before){
            _cycloStore->addSmart(SmartCycloAction_t::FWD);
        }
        break;

    case PrimitiveCycloAction_t::LEFT:
        switch(calcPrimitiveCycloAction(++ind)){
        case PrimitiveCycloAction_t::LEFT:
            _cycloStore->addSmart(SmartCycloAction_t::SS180L);
            break;
        default:
            --ind;
            _cycloStore->addSmart(SmartCycloAction_t::SS90SL);
        }
        break;

    case PrimitiveCycloAction_t::RIGHT:
        switch(calcPrimitiveCycloAction(++ind)){
        case PrimitiveCycloAction_t::RIGHT:
            _cycloStore->addSmart(SmartCycloAction_t::SS180R);
            break;
        default:
            --ind;
            _cycloStore->addSmart(SmartCycloAction_t::SS90SR);
        }
        break;

    case PrimitiveCycloAction_t::STOP:
        _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
        _cycloStore->addSmart(SmartCycloAction_t::STOP);
        break;

    case PrimitiveCycloAction_t::BLANK: 
        _cycloStore->addSmart(SmartCycloAction_t::IDLE);
        break;
    }
}

void Robot::primitiveToExplorer(uint8_t ind)
{
    switch (calcPrimitiveCycloAction(ind))
    {
    case PrimitiveCycloAction_t::FORWARD:
        _cycloStore->addSmart(SmartCycloAction_t::FWD);
        break;

    case PrimitiveCycloAction_t::LEFT:
        _cycloStore->addSmart(SmartCycloAction_t::SS90SL);
        break;

    case PrimitiveCycloAction_t::RIGHT:
        _cycloStore->addSmart(SmartCycloAction_t::SS90SR);
        break;

    case PrimitiveCycloAction_t::STOP:
        _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
        _cycloStore->addSmart(SmartCycloAction_t::STOP);
        break;

    case PrimitiveCycloAction_t::BLANK: 
        _cycloStore->addSmart(SmartCycloAction_t::IDLE);
        break;
    }
}

void Robot::pathToCyclogram(){
    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);  
    for(uint8_t i = 0; i < _Maze->GetPathSize(); i++){
        // calcRelativeCycloAction(i);
        primitiveToFast(i);
    }
    // Serial.println();
}

void Robot::moveFloodFill()
{
    Cell CellFromSensors = _optocoupler->getCellFromSensors(_odometry->getDir());
    _Maze->SetCell(CellFromSensors, _odometry->getMazeCoord());
    _solver->SolveBfsMaze(_odometry->getMazeCoord(), {MAZE_FINISH_CELLS_X, MAZE_FINISH_CELLS_Y});
    primitiveToExplorer(FIRST);
}