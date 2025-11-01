#include "CycloUtilits/ConvertToFasts.h"


#define CONVERTER_OUTPUT 0

#if CONVERTER_OUTPUT
    #define CONVERTER_PRINT(x) Serial.print((x))
    #define CONVERTER_PRINTLN(x) Serial.println((x))
#else
    #define CONVERTER_PRINT(x)
    #define CONVERTER_PRINTLN(x)
#endif

void ConverterToFasts::convert()
{
    #if CONVERTER_OUTPUT
    _cycloStore->printPrimitives();
    #endif
    _cycloStore->addSmart(SmartCycloAction_t::TO_BACK_ALIGN);
    _cycloStore->addSmart(SmartCycloAction_t::FROM_BACK_ALIGN_TO_CENTER);

    PrimitiveCycloAction_t curPrim = PrimitiveCycloAction_t::BLANK;
    
    curPrim = _cycloStore->popFrontPrimitive(); // 0 действие всея конвертера (всегда FORWARD)

    while(curPrim != PrimitiveCycloAction_t::STOP) // проверяем кластеры циклограмм, пока не найдём стоп
    {
        //обработка одного кластера циклограмм
        CONVERTER_PRINTLN("start Claster");
        RobotState_t repeatStartState = _entryHandler();
        CONVERTER_PRINTLN("entry done");
        if(repeatStartState == RobotState_t::ORT) 
        {
            curPrim = _cycloStore->popFrontPrimitive(); // 0 действие следующего кластера(всегда FORWARD или STOP)
            continue; // кластер действий был завершён в entryHandler и repeatHandler не нужен
        }
        //_cycloStore->printPrimitives();

        RobotState_t exitStartState = _repeatActionHandler(repeatStartState);
        CONVERTER_PRINTLN("repeat done");
        //_cycloStore->printPrimitives();

        _exitHandler(exitStartState);
        CONVERTER_PRINTLN("exit done");
        //_cycloStore->printPrimitives();
        
        curPrim = _cycloStore->popFrontPrimitive(); // 0 действие следующего кластера(всегда FORWARD или STOP)
    }
    _cycloStore->printSmarts();
    #if CONVERTER_OUTPUT
    _cycloStore->printPrimitives();
    _cycloStore->printSmarts();
    #endif
}


ConverterToFasts::RobotState_t ConverterToFasts::_entryHandler()
{
    PrimitiveCycloAction_t nextPrim[3];
    nextPrim[0] = _cycloStore->virtualPopFrontPrimitive();
    nextPrim[1] = _cycloStore->virtualPopFrontPrimitive(); // необязательно будет использоваться 
    nextPrim[2] = _cycloStore->virtualPopFrontPrimitive(); // необязательно будет использоваться 
    CONVERTER_PRINTLN(String(toInt(nextPrim[0])) + " " + String(toInt(nextPrim[1])) + " " + String(toInt(nextPrim[2])));
    if(nextPrim[0] == PrimitiveCycloAction_t::FORWARD || nextPrim[0] == PrimitiveCycloAction_t::STOP)
    {
        _cycloStore->virtualGoBack();
        _cycloStore->virtualPopFrontPrimitive(); // это действие мы уже смотрели
        _EH_FWD_X_handler();
        CONVERTER_PRINTLN("afterFWD");
        return RobotState_t::ORT;
    }
    CONVERTER_PRINTLN("before90");
    if(_EH_SS90S_handler(nextPrim)) return RobotState_t::ORT;
    CONVERTER_PRINTLN("before180");
    if(_EH_SS180S_handler(nextPrim)) return RobotState_t::ORT;
    CONVERTER_PRINTLN("before45");
    if(_EH_SD45S_handler(nextPrim)) return RobotState_t::TURN45;
    CONVERTER_PRINTLN("before135");
    if(_EH_SD135S_handler(nextPrim)) return RobotState_t::TURN45;
    
}

void ConverterToFasts::_EH_FWD_X_handler()
{
    uint8_t X = 1; // т.к. определено выше
    for
    (
        PrimitiveCycloAction_t curPrim = _cycloStore->virtualPopFrontPrimitive();
        curPrim == PrimitiveCycloAction_t::FORWARD || curPrim == PrimitiveCycloAction_t::STOP;
        X++
    )
    {
        curPrim = _cycloStore->virtualPopFrontPrimitive();
    }
    CONVERTER_PRINTLN("FWD X:" + String(X));
    _cycloStore->virtualGoBack();
    // релизим все примитивы кроме последнего FWD т.к. он будет взят в начале следующего кластера
    for(uint8_t i = 0; i < X - 1; i++) _cycloStore->virtualPopFrontPrimitive(); 
    _cycloStore->virtualPrimitiveRelease();
    
    _cycloStore->addSmart(SmartCycloAction_t::FWD_X, X);
}

bool ConverterToFasts::_EH_SS90S_handler(PrimitiveCycloAction_t* nextPrim)
{
    PrimitiveCycloAction_t TURN = nextPrim[0];
    PrimitiveCycloAction_t OP_TURN = toOpposite(TURN);

    //если следующее действие не конец кластера, то выходим из функции
    if( !(nextPrim[1] == PrimitiveCycloAction_t::FORWARD || nextPrim[1] == PrimitiveCycloAction_t::STOP))
    {
        CONVERTER_PRINTLN("no90s");
        return false;
    }
    
    CONVERTER_PRINTLN("after90s");
    switch(TURN)
    {
    case PrimitiveCycloAction_t::LEFT:
        _cycloStore->addSmart(SmartCycloAction_t::SS90SL);
        break;
    case PrimitiveCycloAction_t::RIGHT:
        _cycloStore->addSmart(SmartCycloAction_t::SS90SR);
        break;
    }
    _cycloStore->virtualGoBack();
    _cycloStore->popFrontPrimitive(); //мы обработали 2 действия, но второе нужно далее, так что что релизим первое
    return true;
}

bool ConverterToFasts::_EH_SS180S_handler(PrimitiveCycloAction_t* nextPrim)
{
    PrimitiveCycloAction_t TURN = nextPrim[0];
    PrimitiveCycloAction_t OP_TURN = toOpposite(TURN);

    bool nextIsTurn = nextPrim[1] == TURN,
         secondNextIsEndOfClaster = nextPrim[2] == PrimitiveCycloAction_t::FORWARD || nextPrim[2] == PrimitiveCycloAction_t::STOP;
    if(nextIsTurn && secondNextIsEndOfClaster)
    {
        _cycloStore->virtualGoBack();
        _cycloStore->popFrontPrimitive();
        _cycloStore->popFrontPrimitive(); // последнюю релизить не нужно, она FORWARD
        _cycloStore->virtualPrimitiveRelease();
        switch(TURN)
        {
        case PrimitiveCycloAction_t::LEFT:
            _cycloStore->addSmart(SmartCycloAction_t::SS180SL);
            break;
        case PrimitiveCycloAction_t::RIGHT:
            _cycloStore->addSmart(SmartCycloAction_t::SS180SR);
            break;
        }
    }
}

bool ConverterToFasts::_EH_SD45S_handler(PrimitiveCycloAction_t* nextPrim)
{
    PrimitiveCycloAction_t TURN = nextPrim[0];
    PrimitiveCycloAction_t OP_TURN = toOpposite(TURN);

    if(nextPrim[0] == TURN && nextPrim[1] == OP_TURN)
    {
        _cycloStore->virtualGoBack();
        _cycloStore->popFrontPrimitive(); // обработали мы 2 действия, но второе нужно далее, так что что релизим первое
        CONVERTER_PRINTLN("after45");
        switch(TURN)
        {
        case PrimitiveCycloAction_t::LEFT:
            _cycloStore->addSmart(SmartCycloAction_t::SD45SL);
            break;
        case PrimitiveCycloAction_t::RIGHT:
            _cycloStore->addSmart(SmartCycloAction_t::SD45SR);
            break;
        }
        return true;
    }
    return false;
}
bool ConverterToFasts::_EH_SD135S_handler(PrimitiveCycloAction_t* nextPrim)
{
    PrimitiveCycloAction_t TURN = nextPrim[0];
    PrimitiveCycloAction_t OP_TURN = toOpposite(TURN);

    if(nextPrim[0] == TURN && nextPrim[1] == TURN && nextPrim[2] == OP_TURN)
    {
        _cycloStore->virtualGoBack();
        _cycloStore->popFrontPrimitive();
        _cycloStore->popFrontPrimitive(); // обработали мы 3 действия, но третье нужно далее, так что что релизим первое
        CONVERTER_PRINTLN("after135");
        switch(TURN)
        {
        case PrimitiveCycloAction_t::LEFT:
            _cycloStore->addSmart(SmartCycloAction_t::SD135SL);
            break;
        case PrimitiveCycloAction_t::RIGHT:
            _cycloStore->addSmart(SmartCycloAction_t::SD135SR);
            break;
        }
        return true;
    }
    return false;
}


ConverterToFasts::RobotState_t ConverterToFasts::_repeatActionHandler(RobotState_t startState)
{
    //пока нет действия выхода повторяем действия repeatHandler-а
    bool diagFlag = true, dd90sFlag = true;
    do
    {
        dd90sFlag = _RH_DD90S_handler();
        diagFlag = _RH_DIAG_X_handler();
        CONVERTER_PRINTLN("repeat");
    } while(dd90sFlag || diagFlag);
}

bool ConverterToFasts::_RH_DD90S_handler()
{
    PrimitiveCycloAction_t nextPrim[2];
    uint8_t X = 0;
    nextPrim[0] = _cycloStore->virtualPopFrontPrimitive();
    nextPrim[1] = _cycloStore->virtualPopFrontPrimitive();
    PrimitiveCycloAction_t TURN = nextPrim[0];
    PrimitiveCycloAction_t OP_TURN = toOpposite(TURN);
    while(nextPrim[0] == nextPrim[1] && 
        (nextPrim[0] == PrimitiveCycloAction_t::LEFT || nextPrim[0] == PrimitiveCycloAction_t::RIGHT))
    {
        X++;
        nextPrim[0] = _cycloStore->virtualPopFrontPrimitive();
        nextPrim[1] = _cycloStore->virtualPopFrontPrimitive();
    }
    _cycloStore->virtualGoBack();
    
    CONVERTER_PRINTLN("DD X:" + String(X));

    for(int i = 0; i < X - 1; i++) 
    {
        _cycloStore->popFrontPrimitive();
        _cycloStore->popFrontPrimitive(); // релизим всё то, что не мешает DS135S
    }

    PrimitiveCycloAction_t next3Prim[3];
    next3Prim[0] = _cycloStore->virtualPopFrontPrimitive();
    next3Prim[1] = _cycloStore->virtualPopFrontPrimitive();
    next3Prim[2] = _cycloStore->virtualPopFrontPrimitive();
    _cycloStore->virtualGoBack();
    if( //это ситуация выхода в 135 из DD90S
        (next3Prim[0] == PrimitiveCycloAction_t::LEFT || next3Prim[0] == PrimitiveCycloAction_t::RIGHT) &&
        (next3Prim[1] == next3Prim[0]) &&
        (next3Prim[2] == PrimitiveCycloAction_t::FORWARD || next3Prim[2] == PrimitiveCycloAction_t::STOP) 
      )
    {
        CONVERTER_PRINTLN("DD90S exit 135");
        if(X == 1) return false;
        else X--;
    }

    if( //это ситуация выхода в 45 или диагональ из DD90S
        (next3Prim[0] == PrimitiveCycloAction_t::LEFT || next3Prim[0] == PrimitiveCycloAction_t::RIGHT) &&
        (next3Prim[1] == next3Prim[0]) &&
        (next3Prim[2] == toOpposite(next3Prim[0]))
      )
    {//тут нету дополнительных условий, т.к. если сюда мы заходим, значит x минимум 1
        _cycloStore->popFrontPrimitive();
        _cycloStore->popFrontPrimitive();
    }
    if(X == 0) return false;

    CONVERTER_PRINTLN("X: " + String(X));
    for(int i = 0; i < X; i++)
    {
        bool isEven = !(i % 2);
        if(TURN == PrimitiveCycloAction_t::LEFT)
        {
            if(isEven) _cycloStore->addSmart(SmartCycloAction_t::DD90SL);
            else _cycloStore->addSmart(SmartCycloAction_t::DD90SR);
        }
        else
        {
            if(isEven) _cycloStore->addSmart(SmartCycloAction_t::DD90SR);
            else _cycloStore->addSmart(SmartCycloAction_t::DD90SL);
        }
    }
    return true;
}

bool ConverterToFasts::_RH_DIAG_X_handler()
{
    CONVERTER_PRINTLN("RH DIAG_X");
    //_cycloStore->printPrimitives();
    PrimitiveCycloAction_t oldPrim = _cycloStore->virtualPopFrontPrimitive();
    PrimitiveCycloAction_t curPrim = _cycloStore->virtualPopFrontPrimitive();
    //CONVERTER_PRINTLN("cur: " + String(toInt(curPrim)) + " old: " + String(toInt(oldPrim)));

    PrimitiveCycloAction_t TURN = oldPrim;
    PrimitiveCycloAction_t OP_TURN = toOpposite(TURN);

    uint8_t X = 1; // от oldPrim
    while((curPrim == PrimitiveCycloAction_t::LEFT || curPrim == PrimitiveCycloAction_t::RIGHT) 
        && curPrim == toOpposite(oldPrim))
    {
        X++;
        oldPrim = curPrim;
        curPrim = _cycloStore->virtualPopFrontPrimitive();
    }
    _cycloStore->virtualGoBack();

    CONVERTER_PRINTLN("Diag X:" + String(X));
    if(X == 1) return false;

    for(int i = 0; i < X - 1; i++)
    {
        _cycloStore->popFrontPrimitive(); // релизим всё то, что не мешает действиям выхода
    }
    _cycloStore->addSmart(SmartCycloAction_t::DIAG_X, X - 1);

    return true;
}

void ConverterToFasts::_exitHandler(RobotState_t startState)
{
    PrimitiveCycloAction_t nextPrim[3];
    nextPrim[0] = _cycloStore->virtualPopFrontPrimitive();
    nextPrim[1] = _cycloStore->virtualPopFrontPrimitive();
    nextPrim[2] = _cycloStore->virtualPopFrontPrimitive(); // необязательно будет использоваться 

    CONVERTER_PRINTLN("exit start");
    CONVERTER_PRINTLN("nextPrims: " + String(toInt(nextPrim[0])) + " " + String(toInt(nextPrim[1])) + " " + String(toInt(nextPrim[2])));
    
    if(_EXH_DS45S_handler(nextPrim)) return;
    if(_EXH_DS135S_handler(nextPrim)) return;
}

bool ConverterToFasts::_EXH_DS45S_handler(PrimitiveCycloAction_t* nextPrim)
{
    bool nextIsTurn = nextPrim[0] == PrimitiveCycloAction_t::LEFT || nextPrim[0] == PrimitiveCycloAction_t::RIGHT,
         secondNextIsEndOfClaster = nextPrim[1] == PrimitiveCycloAction_t::FORWARD || nextPrim[1] == PrimitiveCycloAction_t::STOP;
    CONVERTER_PRINTLN("bools 45:" + String(nextIsTurn) + " " + String(secondNextIsEndOfClaster));
    if(nextIsTurn && secondNextIsEndOfClaster)
    {
        // обработали мы 2 действия, но второе удаляется в начале кластера, так что что релизим первое
        _cycloStore->virtualGoBack();
        _cycloStore->popFrontPrimitive();

        switch(nextPrim[0])
        {
            CONVERTER_PRINTLN("after45");
        case PrimitiveCycloAction_t::LEFT:
            _cycloStore->addSmart(SmartCycloAction_t::DS45SL);
            break;
        case PrimitiveCycloAction_t::RIGHT:
            _cycloStore->addSmart(SmartCycloAction_t::DS45SR);
            break;
        }

        return true;
    }
    return false;
}

bool ConverterToFasts::_EXH_DS135S_handler(PrimitiveCycloAction_t* nextPrim)
{
    bool nextIsTurn = nextPrim[0] == PrimitiveCycloAction_t::LEFT || nextPrim[0] == PrimitiveCycloAction_t::RIGHT,
         secondNextIsSameTurn = nextPrim[1] == nextPrim[0],
         thirdNextIsEndOfClaster = nextPrim[2] == PrimitiveCycloAction_t::FORWARD || nextPrim[2] == PrimitiveCycloAction_t::STOP;
    
    CONVERTER_PRINTLN("bools 135:" + String(nextIsTurn) + " " + String(secondNextIsSameTurn) + " " + String(thirdNextIsEndOfClaster));
    if(nextIsTurn && secondNextIsSameTurn && thirdNextIsEndOfClaster)
    {
        // обработали мы 3 действия, но третье удаляется в начале кластера, так что что релизим первое
        _cycloStore->virtualGoBack();
        _cycloStore->popFrontPrimitive();
        _cycloStore->popFrontPrimitive();

        switch(nextPrim[0])
        {
            CONVERTER_PRINTLN("after135");
        case PrimitiveCycloAction_t::LEFT:
            _cycloStore->addSmart(SmartCycloAction_t::DS135SL);
            break;
        case PrimitiveCycloAction_t::RIGHT:
            _cycloStore->addSmart(SmartCycloAction_t::DS135SR);
            break;
        }

        return true;
    }
    return false;
}