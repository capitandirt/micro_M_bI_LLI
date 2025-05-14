#include "CycloUtilits/ActionsHandler.h"

const PrimitiveCycloAction_t ActionsHandler::calc_primitive_cyclo_action(const uint8_t ind){
    if(ind >= _maze->GetPathSize() - 1) return PrimitiveCycloAction_t::STOP;

    int8_t dir_now  = static_cast<int8_t>(_maze->GetPathDir(ind));
    int8_t dir_next = static_cast<int8_t>(_maze->GetPathDir(ind + 1));

    return static_cast<PrimitiveCycloAction_t>((dir_now - dir_next + DIRECTION_SIZE) % DIRECTION_SIZE);
}

void ActionsHandler::dirs_to_primitives(){
    for(uint8_t i = 0; i < _maze->GetPathSize(); i++){
        _cycloStore->addPrimitive(calc_primitive_cyclo_action(i));
    }
}

void ActionsHandler::start_explorer_process(Direction robot_dir)
{
    _cycloStore->reloadSmarts();
    
    const int8_t from_robot_dir = static_cast<int8_t>(robot_dir);
    const int8_t from_path_dir  = static_cast<int8_t>(_maze->GetPathDir(0));

    const auto first_primitive = static_cast<PrimitiveCycloAction_t>((from_robot_dir - from_path_dir + DIRECTION_SIZE) % DIRECTION_SIZE);
    switch(first_primitive) // установка соответствия направления робота и направлений пути
    {
        case PrimitiveCycloAction_t::FORWARD:
            _cycloStore->addSmart(SmartCycloAction_t::FWD_X);
            break;

        case PrimitiveCycloAction_t::BACK:
            _cycloStore->addSmart(SmartCycloAction_t::IP90L);
            _cycloStore->addSmart(SmartCycloAction_t::IP90L);
            break;

        default:
            break;
    }
}

void ActionsHandler::loadExplorer(Direction robot_dir){
    reload();

    const auto from_robot_dir = static_cast<int8_t>(robot_dir);
    const auto from_path_dir  = static_cast<int8_t>(_maze->GetPathDir(0));

    const auto first_primitive = static_cast<PrimitiveCycloAction_t>(
        (from_robot_dir - from_path_dir + DIRECTION_SIZE) % DIRECTION_SIZE);

    switch (first_primitive)
    {
    case PrimitiveCycloAction_t::FORWARD:
        _cycloStore->addSmart(SmartCycloAction_t::FWDE);
        break;
    case PrimitiveCycloAction_t::LEFT:
        _cycloStore->addSmart(SmartCycloAction_t::SS90EL);
        break;
    case PrimitiveCycloAction_t::RIGHT:
        _cycloStore->addSmart(SmartCycloAction_t::SS90ER);
        break;
    case PrimitiveCycloAction_t::BACK:
        _cycloStore->addSmart(SmartCycloAction_t::IP180);
    default:
        break;
    }

    _cycloStore->addSmart(SmartCycloAction_t::CLUSTER_DOT);
}

/*useless code*/
void ActionsHandler::primitivesToExplorers(Direction robot_dir)
{
    dirs_to_primitives();
    start_explorer_process(robot_dir);

    while(!_cycloStore->primitiveIsEmpty()){
             if (TO_SS90E());
        else if (TO_FWD_X());
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
        else if (TO_FWD_X());
        else if (TO_STOP());
        else if (TO_IDLE());
    }
}

void ActionsHandler::reload(){
    _cycloStore->reloadPrimitives();
    _cycloStore->reloadSmarts();
}

void ActionsHandler::needStartCellAligning(){
    _cycloStore->addSmart(SmartCycloAction_t::TO_ALIGN);
    _cycloStore->addSmart(SmartCycloAction_t::FROM_ALIGN_TO_CENTER);
    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
    _cycloStore->addSmart(SmartCycloAction_t::CLUSTER_DOT);
}

void ActionsHandler::needGetOutImpasse(){
    _cycloStore->addSmart(SmartCycloAction_t::IP180);
    _cycloStore->addSmart(SmartCycloAction_t::FWD_X);
}

void ActionsHandler::needStop(){
    _cycloStore->reloadSmarts();
    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
    _cycloStore->addSmart(SmartCycloAction_t::STOP);
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

bool ActionsHandler::TO_FWD_X()
{
    if(_cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::FORWARD){
        for(uint8_t X = 0; _cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::FORWARD; X++)
        {
            _cycloStore->addSmart(SmartCycloAction_t::FWD_X);
        }

        
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
        const auto OP_TURN = static_cast<PrimitiveCycloAction_t>((toInt(TURN) + DIRECTION_SIZE/2) % DIRECTION_SIZE); // расчёт противоположного основному поворота
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

ActionsHandler::RobotState_t ActionsHandler::entryHandler()
{
    //=====================================================================================================================
    PrimitiveCycloAction_t curPrim = _cycloStore->virtualPopFrontPrimitive(); // 1 действие
    if(curPrim == PrimitiveCycloAction_t::FORWARD) // обработка 1 действия
    {
        _cycloStore->virtualGoBack(); // это делается чтобы изначально X был равен 0
        return RobotState_t::FORWARD;
    }
    _cycloStore->virtualPrimitiveRelease();
    
    const PrimitiveCycloAction_t TURN = curPrim; // получение действия и приравнивание его к основному повороту
    const PrimitiveCycloAction_t OP_TURN = static_cast<PrimitiveCycloAction_t>((static_cast<int8_t>(TURN) + DIRECTION_SIZE/2) % DIRECTION_SIZE); // расчёт противоположного основному поворота
    
    //=====================================================================================================================
    curPrim = _cycloStore->virtualPopFrontPrimitive(); // 2 действия
    if(curPrim == PrimitiveCycloAction_t::FORWARD) // обработка 2 действия
    {
        switch(TURN)
        {
            case PrimitiveCycloAction_t::LEFT:
                _cycloStore->addSmart(SmartCycloAction_t::SS90SL);
                break;
            case PrimitiveCycloAction_t::RIGHT:
                _cycloStore->addSmart(SmartCycloAction_t::SS90SR);
                break;
        }
        return RobotState_t::STOP;
    }
    else if(curPrim == OP_TURN) // обработка 2 действия
    {
        switch(TURN)
        {
            case PrimitiveCycloAction_t::LEFT:
                _cycloStore->addSmart(SmartCycloAction_t::SD45SL);
                break;
            case PrimitiveCycloAction_t::RIGHT:
                _cycloStore->addSmart(SmartCycloAction_t::SD45SR);
                break;
        }
        _cycloStore->virtualGoBack(); //второе действие доказало, что это 45, но далее в обработчике повторов нам оно будет нужно
        return toState(TURN); //вход в какой-то повтор под углом 45 градусов
    } 
    //если 2 действие TURN:
    //=====================================================================================================================
    _cycloStore->virtualPrimitiveRelease(); //вернём его, если будет 135
    curPrim = _cycloStore->virtualPopFrontPrimitive(); // 3 действие
    if(curPrim == PrimitiveCycloAction_t::FORWARD) // обработка 3 действия
    {
        switch(TURN)
        {
            case PrimitiveCycloAction_t::LEFT:
                _cycloStore->addSmart(SmartCycloAction_t::SS180SL);
                break;
            case PrimitiveCycloAction_t::RIGHT:
                _cycloStore->addSmart(SmartCycloAction_t::SS180SR);
                break;
        }
        _cycloStore->virtualPrimitiveRelease();
        return RobotState_t::STOP;
    }
    else if(curPrim == OP_TURN) // обработка 3 действия 
    {
        switch(TURN)
        {
            case PrimitiveCycloAction_t::LEFT:
                _cycloStore->addSmart(SmartCycloAction_t::SD135SL);
                break;
            case PrimitiveCycloAction_t::RIGHT:
                _cycloStore->addSmart(SmartCycloAction_t::SD135SR);
                break;
        }
        _cycloStore->virtualGoBack(); //третье действие доказало, что это 135, но далее в обработчике повторов нам оно будет нужно
        return toState(TURN); //вход в какой-то повтор под углом 135 градусов
    }
    else 
    {
        return RobotState_t::NAS; //он решил повернуть в стену, это плохо
    }
}



ActionsHandler::RobotState_t ActionsHandler::TO_DD90X(const RobotState_t startState)
{
    int X = 0;

    const auto TURN = _cycloStore->virtualPopFrontPrimitive();
    const auto OP_TURN = static_cast<PrimitiveCycloAction_t>((toInt(TURN) + DIRECTION_SIZE/2) % DIRECTION_SIZE); // расчёт противоположного основному поворота
    PrimitiveCycloAction_t lastTurn = OP_TURN;

    if(TURN != PrimitiveCycloAction_t::LEFT && TURN != PrimitiveCycloAction_t::RIGHT)
    {
        return startState;
    };

    PrimitiveCycloAction_t next2Prim[2];
    next2Prim[0] = TURN;
    next2Prim[1] = _cycloStore->virtualPopFrontPrimitive();
    for(;next2Prim[0] == next2Prim[1] && ((!X%2 && next2Prim[0] == OP_TURN) || (X%2 && next2Prim[0] == TURN)); X++)
    {
        _cycloStore->virtualPrimitiveRelease();
        next2Prim[0] = _cycloStore->virtualPopFrontPrimitive();
        next2Prim[1] = _cycloStore->virtualPopFrontPrimitive();
    }
    _cycloStore->virtualGoBack();
    
    for(int i = 0; i < X; i++)
    {
        if(TURN == PrimitiveCycloAction_t::LEFT && (i % 2) || TURN == PrimitiveCycloAction_t::RIGHT && !(i % 2))
        {
            _cycloStore->addSmart(SmartCycloAction_t::DD90SR);
            lastTurn = PrimitiveCycloAction_t::RIGHT;
        } 
        else
        {
            _cycloStore->addSmart(SmartCycloAction_t::DD90SL);
            lastTurn = PrimitiveCycloAction_t::LEFT;
        }
        return toState(lastTurn);
    } 
    
}

PrimitiveCycloAction_t ActionsHandler::TO_DIA_X()
{
    int X = 0;
}



ActionsHandler::RobotState_t ActionsHandler::repeatActionHandler(const RobotState_t startState)
{ 
    if(startState == RobotState_t::FORWARD)
    {
        TO_FWD_X();
        return RobotState_t::FORWARD;
    }
    // НАПИСАТЬ!!!
}

int ActionsHandler::convertToSmart()
{
    PrimitiveCycloAction_t curPrim = _cycloStore->virtualPopFrontPrimitive(); // 0 действие [всегда FORWARD]
    while(curPrim != PrimitiveCycloAction_t::STOP) // пока не стоит остановка
    {
        RobotState_t repeatStartState = entryHandler();
        if(repeatStartState == RobotState_t::NAS) return -1;// вход вылетел с ошибкой поворота в стену на 3 действии
        
        RobotState_t repeatEndState = repeatActionHandler(repeatStartState);
        //ДОПИСАТЬ!!!
    }
}


// int ActionsHandler::convertToSmart() // не закончена, остановился на обработке 4 действия в случае TURN (на DIA)
// {
//     PrimitiveCycloAction_t curPrim = _cycloStore->virtualPopFrontPrimitive(); // 1 действие [всегда FORWARD]
//     while(curPrim != PrimitiveCycloAction_t::STOP) // пока не стоит остановка
//     {
//         int X = 1;
//         curPrim = _cycloStore->virtualPopFrontPrimitive(); // 2 действие
//         if(curPrim == PrimitiveCycloAction_t::FORWARD) // обработка 2 действия
//         {
//             while(_cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::FORWARD) {X++;}
//             _cycloStore->addSmart(SmartCycloAction_t::FWD_X, X);
//             _cycloStore->virtualPrimitiveRelease();
//         }
//         else // обработка 2 действия
//         {
//             PrimitiveCycloAction_t TURN = curPrim; // получение действия и приравнивание его к основному повороту
//             PrimitiveCycloAction_t OP_TURN = static_cast<PrimitiveCycloAction_t>((static_cast<int8_t>(TURN) + DIRECTION_SIZE/2) % DIRECTION_SIZE); // расчёт противоположного основному поворота
            
//             curPrim = _cycloStore->virtualPopFrontPrimitive(); // 3 действия
            
//             if(curPrim == PrimitiveCycloAction_t::FORWARD) // обработка 3 действия
//             {
//                 if(TURN == PrimitiveCycloAction_t::LEFT) {_cycloStore->addSmart(SmartCycloAction_t::SS90SL);}
//                 else {_cycloStore->addSmart(SmartCycloAction_t::SS90SR);}
//                 _cycloStore->virtualPrimitiveRelease();
//             }
//             else if(curPrim == TURN) // обработка 3 действия
//             {
//                 curPrim = _cycloStore->virtualPopFrontPrimitive(); // 4 действия
//                 if(curPrim == PrimitiveCycloAction_t::FORWARD) // обработка 4 действия
//                 {
//                     if(TURN == PrimitiveCycloAction_t::LEFT) {_cycloStore->addSmart(SmartCycloAction_t::SS180SL);}
//                     else {_cycloStore->addSmart(SmartCycloAction_t::SS180SR);}
//                     _cycloStore->virtualPrimitiveRelease();
//                 }
//                 else if(curPrim == TURN) // обработка 4 действия
//                 {
//                     TO_DIA_X(&curPrim);
//                 }
//                 else if(curPrim == OP_TURN) // обработка 4 действия
//                 {
//                     if(TURN == PrimitiveCycloAction_t::LEFT) {_cycloStore->addSmart(SmartCycloAction_t::SD135SL);}
//                     else {_cycloStore->addSmart(SmartCycloAction_t::SD135SR);}

//                     TO_DD90X(&curPrim); // если выполнилось нечётное количество DD90S, то TURN и OP_TURN меняются местами
//                     if(curPrim == TURN) 
//                     {
//                         TURN = OP_TURN;
//                         OP_TURN = curPrim;
//                     }


//                     curPrim = _cycloStore->virtualPopFrontPrimitive(); // 5 действия
//                     if(curPrim == PrimitiveCycloAction_t::FORWARD)
//                     {
//                         if(TURN == PrimitiveCycloAction_t::LEFT) 
//                         {
//                             _cycloStore->addSmart(SmartCycloAction_t::DS45SR);
//                         }
//                         else 
//                         {
//                             _cycloStore->addSmart(SmartCycloAction_t::DS45SL);
//                         }
//                         _cycloStore->virtualPrimitiveRelease();
//                     }
//                     else if(curPrim == OP_TURN)
//                     {
//                         curPrim = _cycloStore->virtualPopFrontPrimitive(); // 6 действия
//                         if(curPrim == PrimitiveCycloAction_t::FORWARD)
//                         {
//                             if(TURN == PrimitiveCycloAction_t::LEFT) 
//                             {
//                                 _cycloStore->addSmart(SmartCycloAction_t::DS135SR);
//                             }
//                             else 
//                             {
//                                 _cycloStore->addSmart(SmartCycloAction_t::DS135SL);
//                             }
//                             _cycloStore->virtualPrimitiveRelease();
//                         }
//                         else
//                         {
//                             return -1; //ОШИБКА, тут либо не работает TO_DD90X, либо вообще ужас.
//                         }
//                     }
//                 }
//                 else return -1; //он решил повернуть в стену, это плохо
//             }
//         }
//     }
// }