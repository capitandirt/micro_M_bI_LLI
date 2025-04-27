#include "CycloUtilits/ActionsHandler.h"

const PrimitiveCycloAction_t ActionsHandler::calc_primitive_cyclo_action(const uint8_t ind){
    if(ind >= _maze->GetPathSize() - 1) return PrimitiveCycloAction_t::STOP;

    auto dirNow  = static_cast<int8_t>(_maze->GetPathDir(ind));
    auto dirNext = static_cast<int8_t>(_maze->GetPathDir(ind + 1));

    return static_cast<PrimitiveCycloAction_t>((dirNow - dirNext + DIRECTION_SIZE) % DIRECTION_SIZE);
}

void ActionsHandler::dirs_to_primitives(){
    _cycloStore->reloadPrimitives();

    for(uint8_t i = 0; i < _maze->GetPathSize(); i++){
        _cycloStore->addPrimitive(calc_primitive_cyclo_action(i));
    }

    _cycloStore->addPrimitive(PrimitiveCycloAction_t::STOP);
    // _cycloStore->printPrimitives();
}

void ActionsHandler::start_primitive_process()
{
    _cycloStore->reloadSmarts();

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

void ActionsHandler::primitivesToExplorers()
{
    dirs_to_primitives();
    start_primitive_process();

    while(!_cycloStore->primitiveIsEmpty()){
             if (TO_SS90E());
        else if (TO_FWD());
        else if (TO_STOP());
        else if (TO_IDLE());
    }
}

void ActionsHandler::primitivesToFasts()
{
    dirs_to_primitives();
    start_primitive_process();

    while(!_cycloStore->primitiveIsEmpty()){
             if (TO_FROM_DIAGS_TO_OP_DIAGS());
        else if (TO_SS90S());
        else if (TO_FWD());
        else if (TO_STOP());
        else if (TO_IDLE());
    }
}

bool ActionsHandler::TO_IDLE(){
    _cycloStore->popFrontPrimitive();
    _cycloStore->addSmart(SmartCycloAction_t::IDLE);
    return true;
}

bool ActionsHandler::TO_STOP(){
    if(_cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::STOP){
        _cycloStore->addSmart(SmartCycloAction_t::STOP);
        _cycloStore->virtualPrimitiveRelease();
        return true;
    }
    else _cycloStore->virtualGoBack();

    return false;
}

bool ActionsHandler::TO_FWD(){
    if(_cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::FORWARD){
        _cycloStore->addSmart(SmartCycloAction_t::FWD);
        _cycloStore->virtualPrimitiveRelease();
        return true;
    }
    else _cycloStore->virtualGoBack();

    return false;
}

bool ActionsHandler::TO_SS90E(){
    if(_cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::LEFT){
        _cycloStore->addSmart(SmartCycloAction_t::SS90EL);
        _cycloStore->virtualPrimitiveRelease();
        return true;
    }
    else _cycloStore->virtualGoBack();

    if(_cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::RIGHT){
        _cycloStore->addSmart(SmartCycloAction_t::SS90ER);
        _cycloStore->virtualPrimitiveRelease();
        return true;
    }
    else _cycloStore->virtualGoBack();

    return false;
}

bool ActionsHandler::TO_SS90S(){
    if(_cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::LEFT){
        _cycloStore->addSmart(SmartCycloAction_t::SS90EL);
        _cycloStore->virtualPrimitiveRelease();
        return true;
    }
    else _cycloStore->virtualGoBack();

    if(_cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::RIGHT){
        _cycloStore->addSmart(SmartCycloAction_t::SS90ER);
        _cycloStore->virtualPrimitiveRelease();
        return true;
    }
    else _cycloStore->virtualGoBack();

    return false;
}

bool ActionsHandler::TO_FROM_DIAGS_TO_OP_DIAGS(){
    if(_cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::FORWARD){
        const PrimitiveCycloAction_t turn = _cycloStore->virtualPopFrontPrimitive();

        if(turn == PrimitiveCycloAction_t::LEFT || turn == PrimitiveCycloAction_t::RIGHT){
            const PrimitiveCycloAction_t op_turn = _cycloStore->virtualPopFrontPrimitive();

            if(toInt(op_turn) == (toInt(turn) + 2) % 4){
                if(_cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::FORWARD){
                    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
                    
                    if(turn == PrimitiveCycloAction_t::LEFT){
                        _cycloStore->addSmart(SmartCycloAction_t::SD45SL);
                        _cycloStore->addSmart(SmartCycloAction_t::DS45SR);
                    }
                    else{
                        _cycloStore->addSmart(SmartCycloAction_t::SD45SR);
                        _cycloStore->addSmart(SmartCycloAction_t::DS45SL);
                    }

                    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
                    _cycloStore->virtualPrimitiveRelease();
                    
                    return true;
                }
            }
        }
    }

    _cycloStore->virtualGoBack();
    return false;
}