#include "CycloUtilits/ActionsHandler.h"

#define OUTPUT_DEBUG 0

#if OUTPUT_DEBUG
    #define AH_PRINT(x) Serial.print((x))
    #define AH_PRINTLN(x) Serial.println((x))
#else
    #define AH_PRINT(x)
    #define AH_PRINTLN(x)
#endif

PrimitiveCycloAction_t ActionsHandler::calc_primitive_cyclo_action(const uint8_t ind){
    if(ind >= _maze->GetPathSize() - 1) return PrimitiveCycloAction_t::STOP;

    int8_t dir_now  = static_cast<int8_t>(_maze->GetPathDir(ind));
    int8_t dir_next = static_cast<int8_t>(_maze->GetPathDir(ind + 1));

    return static_cast<PrimitiveCycloAction_t>((dir_now - dir_next + DIRECTION_SIZE) % DIRECTION_SIZE);
}

void ActionsHandler::dirs_to_primitives(){
    _cycloStore->addPrimitive(PrimitiveCycloAction_t::FORWARD);
    for(uint8_t i = 0; i < _maze->GetPathSize(); i++){
        _cycloStore->addPrimitive(calc_primitive_cyclo_action(i));
    }
}

void ActionsHandler::loadExplorer(Direction robot_dir, bool is_90e){
    clear();

    const auto from_robot_dir = static_cast<int8_t>(robot_dir);
    const auto from_path_dir  = static_cast<int8_t>(_maze->GetPathDir(0));

    const auto explorer_primitive = static_cast<PrimitiveCycloAction_t>(
        (from_robot_dir - from_path_dir + DIRECTION_SIZE) % DIRECTION_SIZE);

    MazeCommand maze_command = _mazeObserver->getCommand(explorer_primitive);

    switch (explorer_primitive)
    {
    case PrimitiveCycloAction_t::FORWARD:
        _cycloStore->addSmart(SmartCycloAction_t::FWDE);
        break;

    case PrimitiveCycloAction_t::LEFT:
        if(!is_90e){
            if(maze_command == MazeCommand::ALIGN_IN_TURN){
                _cycloStore->addSmart(SmartCycloAction_t::TO_FORWARD_ALIGN);
                _cycloStore->addSmart(SmartCycloAction_t::FROM_FORWARD_ALIGN_TO_CENTER);
                _cycloStore->addSmart(SmartCycloAction_t::IP90L);
                _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
            }
            else {
                _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
                _cycloStore->addSmart(SmartCycloAction_t::IP90L);
                _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
            }
        }
        else
            _cycloStore->addSmart(SmartCycloAction_t::SS90EL);
        break;

    case PrimitiveCycloAction_t::RIGHT:
        if(!is_90e){
            if(maze_command == MazeCommand::ALIGN_IN_TURN){
                _cycloStore->addSmart(SmartCycloAction_t::TO_FORWARD_ALIGN);
                _cycloStore->addSmart(SmartCycloAction_t::FROM_FORWARD_ALIGN_TO_CENTER);
                _cycloStore->addSmart(SmartCycloAction_t::IP90R);
                _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
            }
            else {
                _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
                _cycloStore->addSmart(SmartCycloAction_t::IP90R);
                _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
            }
        }
        else
            _cycloStore->addSmart(SmartCycloAction_t::SS90ER);
        break;

    case PrimitiveCycloAction_t::BACK:
        if(maze_command == MazeCommand::ALIGN_IN_IP180){
            _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
            _cycloStore->addSmart(SmartCycloAction_t::TO_FORWARD_ALIGN);
            _cycloStore->addSmart(SmartCycloAction_t::FROM_FORWARD_ALIGN_TO_CENTER);
            _cycloStore->addSmart(SmartCycloAction_t::IP180);
            _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);   
        }
        else _cycloStore->addSmart(SmartCycloAction_t::IP180);
        break;

    default:
        break;
    }

    _cycloStore->addSmart(SmartCycloAction_t::CLUSTER_DOT);
}

void ActionsHandler::primitivesToFasts()
{
    _cycloStore->addSmart(SmartCycloAction_t::TO_BACK_ALIGN);
    _cycloStore->addSmart(SmartCycloAction_t::FROM_BACK_ALIGN_TO_CENTER);
    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);

    dirs_to_primitives();
    _cycloStore->popFrontPrimitive();


    while(_cycloStore->virtualPopFrontPrimitive() != PrimitiveCycloAction_t::STOP){
        _cycloStore->virtualGoBack();
             if (TO_SS90E());
        else if (TO_FWD_X_TEMPLATE());
        else if (TO_STOP());
        else if (TO_IDLE());
    }

    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
}

void ActionsHandler::clear(){
    _cycloStore->reloadPrimitives();
    _cycloStore->reloadSmarts();
}

void ActionsHandler::needStartCellAligning(){
    _cycloStore->reloadSmarts();
    
    _cycloStore->addSmart(SmartCycloAction_t::TO_BACK_ALIGN);
    _cycloStore->addSmart(SmartCycloAction_t::FROM_BACK_ALIGN_TO_CENTER);
    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
    _cycloStore->addSmart(SmartCycloAction_t::CLUSTER_DOT);
}

Direction ActionsHandler::needTurn(Direction dir){
    _cycloStore->reloadSmarts();

    _cycloStore->addSmart(SmartCycloAction_t::TO_FORWARD_ALIGN);
    _cycloStore->addSmart(SmartCycloAction_t::FROM_FORWARD_ALIGN_TO_CENTER);

    _cycloStore->addSmart(SmartCycloAction_t::IP90L);
    _cycloStore->addSmart(SmartCycloAction_t::DELAY_025S);
    _cycloStore->addSmart(SmartCycloAction_t::CLUSTER_DOT);

    Direction next_dir = decDir(dir);
    return next_dir;
}

Direction ActionsHandler::needDirection(const Direction cur,  const Direction need){
    _cycloStore->reloadSmarts();

    const PrimitiveCycloAction_t primitive = static_cast<PrimitiveCycloAction_t>(
        (toInt(need) - toInt(cur) + DIRECTION_SIZE) % DIRECTION_SIZE);

    switch (primitive)
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

    default:
        break;
    }

    _cycloStore->addSmart(SmartCycloAction_t::CLUSTER_DOT);
}

void ActionsHandler::needClusterDot(){
    _cycloStore->reloadSmarts();
    _cycloStore->addSmart(SmartCycloAction_t::CLUSTER_DOT);
}

void ActionsHandler::needEnd(){
    _cycloStore->reloadSmarts();
    _cycloStore->addSmart(SmartCycloAction_t::IP180);
    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
}

void ActionsHandler::needFwdHalf(){
    _cycloStore->reloadSmarts();
    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
    _cycloStore->addSmart(SmartCycloAction_t::CLUSTER_DOT);
}

void ActionsHandler::needDelay05(){
    _cycloStore->addSmart(SmartCycloAction_t::DELAY_025S, 2);
    _cycloStore->addSmart(SmartCycloAction_t::CLUSTER_DOT);
}

void ActionsHandler::needIdle(){
    _cycloStore->reloadSmarts();
    _cycloStore->addSmart(SmartCycloAction_t::IDLE);
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

bool ActionsHandler::TO_FWD_X_TEMPLATE()
{
    if(_cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::FORWARD){
        _cycloStore->virtualPrimitiveRelease(); 
        uint8_t X = 1;
        for(; _cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::FORWARD; X++)
        {
            _cycloStore->virtualPrimitiveRelease();
        }
        _cycloStore->virtualGoBack();
        
        _cycloStore->addSmart(SmartCycloAction_t::FWD_X, X);
        
        
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


uint8_t ActionsHandler::toIntFromState(const ActionsHandler::RobotState_t rs)
{
    return static_cast<uint8_t>(rs);
}

ActionsHandler::RobotState_t ActionsHandler::toState(const PrimitiveCycloAction_t curPrim)
{
    switch(curPrim)
    {
    case PrimitiveCycloAction_t::LEFT:
        return RobotState_t::LEFT;
        
    case PrimitiveCycloAction_t::RIGHT:
        return RobotState_t::RIGHT;

    case PrimitiveCycloAction_t::FORWARD:
        return RobotState_t::FORWARD;
    default:
        return RobotState_t::NAS;// этого не может быть
    }
}

PrimitiveCycloAction_t ActionsHandler::fromState(const RobotState_t rs)
{
    switch(rs)
    {
    case RobotState_t::LEFT:
        return PrimitiveCycloAction_t::LEFT;
    case RobotState_t::FORWARD:
        return PrimitiveCycloAction_t::FORWARD;
    case RobotState_t::RIGHT:
        return PrimitiveCycloAction_t::RIGHT;
    case RobotState_t::STOP:
        return PrimitiveCycloAction_t::STOP;
    default:
        return PrimitiveCycloAction_t::BLANK;
    }
}

void ActionsHandler::loadFasts(){
    _cycloStore->reloadSmarts();
    dirs_to_primitives();
    primitivesToFasts();
}


#undef OUTPUT_DEBUG