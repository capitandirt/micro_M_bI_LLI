#include "CycloUtilits/ActionsHandler.h"

const PrimitiveCycloAction_t ActionsHandler::calc_primitive_cyclo_action(const uint8_t ind){
    if(ind >= _maze->GetPathSize() - 1) return PrimitiveCycloAction_t::STOP;

    int8_t dirNow  = static_cast<int8_t>(_maze->GetPathDir(ind));
    int8_t dirNext = static_cast<int8_t>(_maze->GetPathDir(ind + 1));

    return static_cast<PrimitiveCycloAction_t>((dirNow - dirNext + DIRECTION_SIZE) % DIRECTION_SIZE);
}

void ActionsHandler::dirs_to_primitives(){
    _cycloStore->reloadPrimitives();

    for(uint8_t i = 0; i < _maze->GetPathSize(); i++){
        _cycloStore->addPrimitive(calc_primitive_cyclo_action(i));
    }

    _cycloStore->addPrimitive(PrimitiveCycloAction_t::STOP);
}

void ActionsHandler::start_explorer_process(Direction robot_dir)
{
    _cycloStore->reloadSmarts();
    
    const int8_t from_path_dir = static_cast<int8_t>(_maze->GetPathDir(0));
    const int8_t from_robot_dir = static_cast<int8_t>(robot_dir);

    const auto first_primitive = static_cast<PrimitiveCycloAction_t>((from_robot_dir - from_path_dir + DIRECTION_SIZE) % DIRECTION_SIZE);
    switch(first_primitive) // установка соответствия направления робота и направлений пути
    {
        case PrimitiveCycloAction_t::LEFT:
            _cycloStore->addSmart(SmartCycloAction_t::IP90L);
            break;
        case PrimitiveCycloAction_t::RIGHT:
            _cycloStore->addSmart(SmartCycloAction_t::IP90R);
            break;
        case PrimitiveCycloAction_t::BACK:
            _cycloStore->addSmart(SmartCycloAction_t::IP180);
            break;
        case PrimitiveCycloAction_t::FORWARD:
            break;
        default:
            break;
    }
    
    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
}

void ActionsHandler::primitivesToExplorers(Direction robot_dir)
{
    dirs_to_primitives();
    start_explorer_process(robot_dir);

    while(!_cycloStore->primitiveIsEmpty()){
             if (TO_SS90E());
        else if (TO_FWD());
        else if (TO_STOP());
        else if (TO_IDLE());
    }
}

// !UNWORKABLE CODE
void ActionsHandler::primitivesToFasts()
{
    dirs_to_primitives();
    // start_primitive_process();

    while(!_cycloStore->primitiveIsEmpty()){
             if (TO_SD45S_DS45S());
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
        _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
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

bool ActionsHandler::TO_SD45S_DS45S(){
    if(_cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::FORWARD){
        const PrimitiveCycloAction_t turn = _cycloStore->virtualPopFrontPrimitive();
        
        if(turn == PrimitiveCycloAction_t::LEFT || turn == PrimitiveCycloAction_t::RIGHT){
            const PrimitiveCycloAction_t op_turn = _cycloStore->virtualPopFrontPrimitive();

            if(toInt(op_turn) == (toInt(turn) + 2) % DIRECTION_SIZE){
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
                    _cycloStore->virtualPrimitiveRelease();
                    
                    return true;
                }
            }
        }
    }
    _cycloStore->virtualGoBack();
    return false;
}

bool ActionsHandler::TO_SD135S_DS45S()
{
    if(_cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::FORWARD) // проверка на то, является ли первое действие проездом вперёд
    {
        const PrimitiveCycloAction_t TURN = _cycloStore->virtualPopFrontPrimitive(); // получение второго действия и приравнивание его к основному повороту
        const auto OP_TURN = static_cast<PrimitiveCycloAction_t>((static_cast<int8_t>(TURN) + DIRECTION_SIZE/2) % DIRECTION_SIZE); // расчёт противоположного основному поворота
        if(TURN == PrimitiveCycloAction_t::LEFT || TURN == PrimitiveCycloAction_t::RIGHT) // проверка на то, является ли второе дествие поворотом вообще
        {
            if(_cycloStore->virtualPopFrontPrimitive() == TURN)
            if(_cycloStore->virtualPopFrontPrimitive() == TURN)
            if(_cycloStore->virtualPopFrontPrimitive() == OP_TURN)
            if(_cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::FORWARD)
            {
                if(TURN == PrimitiveCycloAction_t::LEFT)
                {
                    _cycloStore->addSmart(SmartCycloAction_t::SD135SL);
                    _cycloStore->addSmart(SmartCycloAction_t::DS45SR);
                }
                else
                {
                    _cycloStore->addSmart(SmartCycloAction_t::SD135SR);
                    _cycloStore->addSmart(SmartCycloAction_t::DS45SL);
                }
                _cycloStore->virtualPrimitiveRelease();
                return true;
            }
        }
    }
    _cycloStore->virtualGoBack();
    return false;
}