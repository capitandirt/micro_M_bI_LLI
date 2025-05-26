#include "CycloUtilits/ActionsHandler.h"

#define OUTPUT_DEBUG 1

#if OUTPUT_DEBUG
    #define SERIAL_PRINT(x) Serial.print((x))
    #define SERIAL_PRINTLN(x) Serial.println((x))
#else
    #define SERIAL_PRINT(x)
    #define SERIAL_PRINTLN(x)
#endif

PrimitiveCycloAction_t ActionsHandler::calc_primitive_cyclo_action(const uint8_t ind){
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

void ActionsHandler::loadExplorer(Direction robot_dir){
    clear();

    const auto from_robot_dir = static_cast<int8_t>(robot_dir);
    const auto from_path_dir  = static_cast<int8_t>(_maze->GetPathDir(0));

    const auto first_primitive = static_cast<PrimitiveCycloAction_t>(
        (from_robot_dir - from_path_dir + DIRECTION_SIZE) % DIRECTION_SIZE);

    MazeCommand maze_command = _mazeObserver->getCommand(first_primitive);

    switch (first_primitive)
    {
    case PrimitiveCycloAction_t::FORWARD:
        _cycloStore->addSmart(SmartCycloAction_t::FWDE);
        break;

    case PrimitiveCycloAction_t::LEFT:
        if(maze_command == MazeCommand::ALIGN_IN_TURN){
            _cycloStore->addSmart(SmartCycloAction_t::TO_FORWARD_ALIGN);
            _cycloStore->addSmart(SmartCycloAction_t::FROM_FORWARD_ALIGN_TO_CENTER);
            _cycloStore->addSmart(SmartCycloAction_t::IP90L);
            _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
        }
        else _cycloStore->addSmart(SmartCycloAction_t::SS90EL);
        break;

    case PrimitiveCycloAction_t::RIGHT:
        if(maze_command == MazeCommand::ALIGN_IN_TURN){
            _cycloStore->addSmart(SmartCycloAction_t::TO_FORWARD_ALIGN);
            _cycloStore->addSmart(SmartCycloAction_t::FROM_FORWARD_ALIGN_TO_CENTER);
            _cycloStore->addSmart(SmartCycloAction_t::IP90R);
            _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
        }
        else _cycloStore->addSmart(SmartCycloAction_t::SS90ER);
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

/*useless code*/
void ActionsHandler::primitivesToExplorers(Direction robot_dir)
{
    dirs_to_primitives();

    while(!_cycloStore->primitiveIsEmpty()){
             if (TO_SS90E());
        else if (TO_FWD_X());
        else if (TO_STOP());
        else if (TO_IDLE());
    }
}

void ActionsHandler::clear(){
    _cycloStore->reloadPrimitives();
    _cycloStore->reloadSmarts();
}

void ActionsHandler::needStartCellAligning(){
    _cycloStore->addSmart(SmartCycloAction_t::TO_BACK_ALIGN);
    _cycloStore->addSmart(SmartCycloAction_t::FROM_BACK_ALIGN_TO_CENTER);
    _cycloStore->addSmart(SmartCycloAction_t::FWD_HALF);
    _cycloStore->addSmart(SmartCycloAction_t::CLUSTER_DOT);
}

Direction ActionsHandler::needTurn(Direction dir){
    _cycloStore->addSmart(SmartCycloAction_t::IP90L);
    _cycloStore->addSmart(SmartCycloAction_t::DELAY_025S);
    _cycloStore->addSmart(SmartCycloAction_t::CLUSTER_DOT);

    Direction next_dir = decDir(dir);
    return next_dir;
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
    _cycloStore->reloadSmarts();
    _cycloStore->addSmart(SmartCycloAction_t::DELAY_025S);
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
        uint8_t X = 1;
        for(; _cycloStore->virtualPopFrontPrimitive() == PrimitiveCycloAction_t::FORWARD; X++)
        {
            _cycloStore->virtualPrimitiveRelease();
        }
        _cycloStore->virtualGoBack();
        
        _cycloStore->addSmart(SmartCycloAction_t::FWD_X, X);
        SERIAL_PRINTLN("X= " + String(X));
        
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
        const auto OP_TURN = toOpposite(TURN);
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

uint8_t ActionsHandler::toIntFromState(const ActionsHandler::RobotState_t rs)
{
    return static_cast<uint8_t>(rs);
}

ActionsHandler::RobotState_t ActionsHandler::toState(const PrimitiveCycloAction_t curPrim)
{
    SERIAL_PRINT("to_State: ");
    switch(curPrim)
    {
    case PrimitiveCycloAction_t::LEFT:
        SERIAL_PRINTLN("LEFT");
        return RobotState_t::LEFT;
        
    case PrimitiveCycloAction_t::RIGHT:
        SERIAL_PRINTLN("RIGHT");
        return RobotState_t::RIGHT;

    case PrimitiveCycloAction_t::FORWARD:
        SERIAL_PRINTLN("FORWARD");
        return RobotState_t::FORWARD;
    default:
        SERIAL_PRINTLN("error in toState");
        return RobotState_t::NAS;// этого не может быть
    }

    SERIAL_PRINT("\n");
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

ActionsHandler::RobotState_t ActionsHandler::entryHandler()
{
    static int entryCounter = 0;
    entryCounter++;
    //=====================================================================================================================
    PrimitiveCycloAction_t curPrim = _cycloStore->virtualPopFrontPrimitive(); // 1 действие
    SERIAL_PRINTLN("act1: " + String(toInt(curPrim)));
    if(curPrim == PrimitiveCycloAction_t::FORWARD || curPrim == PrimitiveCycloAction_t::STOP) // обработка 1 действия
    {
        _cycloStore->virtualGoBack(); // это делается чтобы изначально X был равен 0
        SERIAL_PRINTLN("exit with FWD");
        return RobotState_t::FORWARD;
    }
    _cycloStore->virtualPrimitiveRelease();
    
    const PrimitiveCycloAction_t TURN = curPrim; // получение действия и приравнивание его к основному повороту
    const PrimitiveCycloAction_t OP_TURN = toOpposite(TURN);
    
    //=====================================================================================================================
    curPrim = _cycloStore->virtualPopFrontPrimitive(); // 2 действия
    SERIAL_PRINTLN("act2: " + String(toInt(curPrim)));
    if(curPrim == PrimitiveCycloAction_t::FORWARD || curPrim == PrimitiveCycloAction_t::STOP) // обработка 2 действия
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
        SERIAL_PRINTLN("exit with SS90S");
        _cycloStore->virtualGoBack();
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
        SERIAL_PRINT("exit with SD45S ");
        SERIAL_PRINT(entryCounter);
        SERIAL_PRINT(" TURNAACTION: ");
        SERIAL_PRINTLN(toInt(TURN));
        return toState(TURN); //вход в какой-то повтор под углом 45 градусов
    } 
    //если 2 действие TURN:
    //=====================================================================================================================
    _cycloStore->virtualPrimitiveRelease(); //вернём его, если будет 135
    curPrim = _cycloStore->virtualPopFrontPrimitive(); // 3 действие
    SERIAL_PRINTLN("act3: " + String(toInt(curPrim)));
    if(curPrim == PrimitiveCycloAction_t::FORWARD || curPrim == PrimitiveCycloAction_t::STOP) // обработка 3 действия
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
        _cycloStore->virtualGoBack();
        SERIAL_PRINTLN("exit with SS180S");
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
        SERIAL_PRINTLN("exit with SD135S " + String(entryCounter) + " primAction: " + toInt(TURN));
        //SERIAL_PRINTLN("entryHandler " + String(toIntFromState(toState(TURN))) + " 351"); 
        return toState(TURN); //вход в какой-то повтор под углом 135 градусов
    }
    else 
    {
        SERIAL_PRINTLN("error in entryHandler, counter is: " + String(entryCounter)); 
        return RobotState_t::NAS; //он решил повернуть в стену, это плохо
    }
}

ActionsHandler::RobotState_t ActionsHandler::TO_DD90X(const RobotState_t startState)
{
    int X = 1;

    const auto TURN = _cycloStore->virtualPopFrontPrimitive();
    const auto OP_TURN = toOpposite(TURN);
    PrimitiveCycloAction_t lastTurn = OP_TURN;

    if(TURN != PrimitiveCycloAction_t::LEFT && TURN != PrimitiveCycloAction_t::RIGHT)
    {
        SERIAL_PRINTLN("err in EH");
        return startState;
    };

    PrimitiveCycloAction_t next2Prim[2];
    next2Prim[0] = TURN;
    next2Prim[1] = _cycloStore->virtualPopFrontPrimitive();
    
    for(;next2Prim[0] == next2Prim[1] && ((!(X%2) && next2Prim[0] == OP_TURN) || (X%2 && next2Prim[0] == TURN)); X++)
    {
        SERIAL_PRINTLN("X: " + String(X));
        SERIAL_PRINTLN("next1: " + String(toInt(next2Prim[0])));
        SERIAL_PRINTLN("next2: " + String(toInt(next2Prim[1])));
        next2Prim[0] = _cycloStore->virtualPopFrontPrimitive();
        next2Prim[1] = _cycloStore->virtualPopFrontPrimitive();
    }
    SERIAL_PRINTLN("end for, X: " + String(X));
    PrimitiveCycloAction_t curPrim = _cycloStore->virtualPopFrontPrimitive();
    if(curPrim != PrimitiveCycloAction_t::LEFT && curPrim != PrimitiveCycloAction_t::RIGHT)
    {
        SERIAL_PRINTLN("x--");
        X--; // это выход в DS135S
    }
    _cycloStore->virtualGoBack();
    
    SERIAL_PRINTLN("final X: " + String(X));
    
    #if OUTPUT_DEBUG 
    _cycloStore->printPrimitives();
    #endif
    if(TURN == PrimitiveCycloAction_t::LEFT) // добавляем 1 действие, которое определили ещё в entryHandler
    {
        _cycloStore->addSmart(SmartCycloAction_t::DD90SR);
        lastTurn = PrimitiveCycloAction_t::RIGHT;
    } 
    else
    {
        _cycloStore->addSmart(SmartCycloAction_t::DD90SL);
        lastTurn = PrimitiveCycloAction_t::LEFT;
    }
    for(int i = 1; i < X; i++)
    {
        _cycloStore->popFrontPrimitive();
        _cycloStore->popFrontPrimitive();
        if(TURN == PrimitiveCycloAction_t::LEFT && !(i % 2) || TURN == PrimitiveCycloAction_t::RIGHT && (i % 2))
        {
            _cycloStore->addSmart(SmartCycloAction_t::DD90SR);
            lastTurn = PrimitiveCycloAction_t::RIGHT;
        } 
        else
        {
            _cycloStore->addSmart(SmartCycloAction_t::DD90SL);
            lastTurn = PrimitiveCycloAction_t::LEFT;
        }
        #if OUTPUT_DEBUG 
        _cycloStore->printPrimitives();
        #endif
    } 
    return toState(lastTurn);
}

ActionsHandler::RobotState_t ActionsHandler::TO_DIA_X(const RobotState_t startState)
{
    PRINTLN("DIA_X");
    #if OUTPUT_DEBUG 
    _cycloStore->printPrimitives();
    #endif
    int X = 2;

    PrimitiveCycloAction_t curPrim = _cycloStore->virtualPopFrontPrimitive();
    PrimitiveCycloAction_t oldPrim = fromState(startState);
    const auto TURN = curPrim;
    const auto OP_TURN = toOpposite(TURN);
    PRINTLN("TURN: " + String(toInt(TURN)));
    if(TURN != PrimitiveCycloAction_t::LEFT && TURN != PrimitiveCycloAction_t::RIGHT)
    {
        _cycloStore->virtualGoBack();
        SERIAL_PRINTLN("exit no found DIA_X");
        return startState;
    };
    while(curPrim == toOpposite(oldPrim))
    {
        curPrim = _cycloStore->virtualPopFrontPrimitive();
        X++;
        oldPrim = curPrim;
        PRINTLN("X: " + String(X));
    }
    _cycloStore->virtualGoBack();
    for(int i = 0; i < X - 3; i++) _cycloStore->virtualPopFrontPrimitive();
    _cycloStore->virtualPrimitiveRelease();
    _cycloStore->addSmart(SmartCycloAction_t::DIAG_X, X);
    
    #if OUTPUT_DEBUG 
    _cycloStore->printPrimitives();
    #endif
    if(!(X % 2)) return startState;
    else return startState == RobotState_t::LEFT ? RobotState_t::RIGHT : RobotState_t::LEFT;
}



ActionsHandler::RobotState_t ActionsHandler::repeatActionHandler(const RobotState_t startState)
{ 
    SERIAL_PRINTLN("RH start: ");
    if(startState == RobotState_t::FORWARD)
    {
        TO_FWD_X();
        SERIAL_PRINTLN("exit FWD_X");
        return RobotState_t::FORWARD;
    }

    PrimitiveCycloAction_t curPrim = _cycloStore->virtualPopFrontPrimitive(); // 1 действие
    SERIAL_PRINTLN(toInt(curPrim));
    PrimitiveCycloAction_t nextPrim = _cycloStore->virtualPopFrontPrimitive(); // 2 действие
    SERIAL_PRINTLN(toInt(nextPrim));


    const auto TURN = curPrim;
    const auto OP_TURN = toOpposite(TURN);
    if(TURN == PrimitiveCycloAction_t::FORWARD)
    {
        SERIAL_PRINTLN("error in RH");
        return RobotState_t::NAS; //это ошибка entryHandler-а
    }
    if(nextPrim == TURN)
    {
        _cycloStore->virtualPrimitiveRelease();
        SERIAL_PRINTLN("to DD90S");
        return TO_DD90X(startState); // на этот момент X = 1
    }
    else if(nextPrim == OP_TURN)
    {
        _cycloStore->virtualPrimitiveRelease();
        SERIAL_PRINTLN("to DIA");
        return TO_DIA_X(startState); // на этот момент X = 2
    } 
    else 
    {
        SERIAL_PRINTLN("no repeat");
        return startState;
    }
}

ActionsHandler::RobotState_t ActionsHandler::exitHandler(const RobotState_t startState)
{
    int entryCounter = 0;
    entryCounter++;
    if(startState == RobotState_t::FORWARD) 
    {
        SERIAL_PRINTLN("exit FWD");
        return RobotState_t::STOP;
    }
    PrimitiveCycloAction_t curPrim = _cycloStore->popFrontPrimitive(); // 1 действие
    SERIAL_PRINTLN("act1: " + String(toInt(curPrim)));
    const auto TURN = curPrim;
    const auto OP_TURN = toOpposite(TURN);
    if(TURN != PrimitiveCycloAction_t::LEFT && TURN != PrimitiveCycloAction_t::RIGHT)
    {
        SERIAL_PRINTLN("TURN err EXH, counter is: " + String(entryCounter));
        return RobotState_t::NAS; //это ошибка, такого быть не должно
    }
    curPrim = _cycloStore->virtualPopFrontPrimitive(); // 2 действие
    SERIAL_PRINTLN("act2: " + String(toInt(curPrim)));
    if(curPrim == TURN) // обработка 2 действия
    {
        curPrim = _cycloStore->virtualPopFrontPrimitive(); // 3 действие
        SERIAL_PRINTLN("act3: " + String(toInt(curPrim)) + " 2nd action is TURN");
        if(curPrim == PrimitiveCycloAction_t::FORWARD || curPrim == PrimitiveCycloAction_t::STOP)
        {
            switch(TURN)
            {
                case PrimitiveCycloAction_t::LEFT:
                    _cycloStore->addSmart(SmartCycloAction_t::DS135SL);
                    break;
                case PrimitiveCycloAction_t::RIGHT:
                    _cycloStore->addSmart(SmartCycloAction_t::DS135SR);
                    break;
            }
            _cycloStore->virtualGoBack();
            _cycloStore->virtualPopFrontPrimitive();
            _cycloStore->virtualPrimitiveRelease(); //мы записываем все действия, кроме последнего форварда
            SERIAL_PRINTLN("exit DS135S");
            return RobotState_t::STOP;
        }
        else if(curPrim == TURN) 
        {
            SERIAL_PRINTLN("error in EXH, counter: " + String(entryCounter));
            return RobotState_t::NAS; // он решил повернуть в стену, это плохо
        }
        else
        {
            _cycloStore->virtualGoBack();
            return startState; //мы наткнулись на DD90X, идём обратно в repeatActionHandler;
        }
    }
    else if(curPrim == PrimitiveCycloAction_t::FORWARD || curPrim == PrimitiveCycloAction_t::STOP) // обработка 2 действия
    {
        _cycloStore->virtualGoBack();
        switch(TURN)
        {
            case PrimitiveCycloAction_t::LEFT:
                _cycloStore->addSmart(SmartCycloAction_t::DS45SL);
                break;
            case PrimitiveCycloAction_t::RIGHT:
                _cycloStore->addSmart(SmartCycloAction_t::DS45SR);
                break;
        }
        _cycloStore->virtualPrimitiveRelease();
        SERIAL_PRINTLN("exit DS45S");
        return RobotState_t::STOP;
    }
    else // OP_TURN обработка 2 действия 
    {
        _cycloStore->virtualGoBack();
        return startState; //мы наткнулись на DIAG_X, идём обратно в repeatActionHandler;
    }
}

void ActionsHandler::convertToSmart()
{
    PrimitiveCycloAction_t curPrim = PrimitiveCycloAction_t::BLANK;
    
    while(curPrim != PrimitiveCycloAction_t::STOP) // пока не стоит остановка
    {
        curPrim = _cycloStore->popFrontPrimitive(); // 0 действие [всегда FORWARD]
        SERIAL_PRINTLN("act0: " + String(toInt(curPrim)));
        SERIAL_PRINTLN("BEFORE ENTRY");
        RobotState_t repeatStartState = entryHandler();

        if(repeatStartState == RobotState_t::NAS)
        {
            SERIAL_PRINTLN("Error in EH/EXH");
            return;
        };// вход вылетел с ошибкой поворота в стену на 3 действии
        SERIAL_PRINTLN("AFTER ENTRY");
        if(repeatStartState == RobotState_t::STOP) continue; // кластер действий был завершён в entryHandler и повтор не нужен
        RobotState_t exitState;
        do
        {
            RobotState_t repeatEndState = repeatActionHandler(repeatStartState);
            SERIAL_PRINTLN(" exit RH ");
            exitState = exitHandler(repeatEndState);
            if(exitState == RobotState_t::NAS)
            {
                SERIAL_PRINTLN("Error in RH");
                return; 
            };// вылетел на моменте, когда turn оказался не turn-ом

            SERIAL_PRINTLN(" exit EXH ");
            repeatStartState = exitState;
        } while (exitState != RobotState_t::STOP);
        
    }
}

#undef OUTPUT_DEBUG