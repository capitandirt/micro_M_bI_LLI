#ifndef _CYCLOGRAMS_H_
#define _CYCLOGRAMS_H_

#include "../../Config.h"
#include "../CycloTypes.h"
#include "../../OptocouplerSensors.h"
#include "Cyclogram_config.h"
/* РАСШИФРОВКА НАЗВАНИЙ ПОВОРОТОВ
SS90EL
S - from straight (0 / 90... градусов поворот) или D - from diagonal
S - to straight или to diagonal
90 - градус поворота
E - explorer или S - smooth
L - левый или R - правый
*/
//исключение - IP90 - In Place

CYCLOGRAM(STOP)
{
    ms->v_f0 = 0;
    ms->theta_i0 = 0;
    ms->isComplete = false;
}

CYCLOGRAM(IDLE)
{
    ms->v_f0 = 0;
    ms->theta_i0 = 0;
    ms->isComplete = true;
}

CYCLOGRAM(FWD_HALF)
{
    ms->v_f0 = FORWARD_SPEED;
    ms->theta_i0 = 0;
    
    int32_t left_sense = s->optocoupler->getSense().left;
    int32_t right_sense = s->optocoupler->getSense().right;
    Cell cell_from_sensors = s->optocoupler->getRelativeCell();

    // регулятор на положение по горизонтали при движении вперёд
    uint8_t regulatorState = toBool(cell_from_sensors.west_wall) << 1 | toBool(cell_from_sensors.east_wall);

    const int32_t LEFT_TRASHHOLD = s->optocoupler->SENSE_THRESHOLD_LEFT;
    const int32_t RIGHT_TRASHHOLD = s->optocoupler->SENSE_THRESHOLD_RIGHT;

    const float regulatorArray[4] = {
        0,//ни один не видит стену
        ANGLLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * (right_sense - RIGHT_TRASHHOLD),//стену видит только правый
        ANGLLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * (LEFT_TRASHHOLD - left_sense),//стену видит только левый
        ANGLLE_SPEED_OPTOCOUPLER_TWOSEN_REG_K * (right_sense - left_sense),//оба датчика  
    };

    ms->theta_i0 = regulatorArray[regulatorState];

    if(s->robotState->getDist() > HALF(CELL_SIZE))
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(FWD)
{
    ms->v_f0 = FORWARD_SPEED;
    ms->theta_i0 = 0;
    
    int32_t left_sense = s->optocoupler->getSense().left;
    int32_t right_sense = s->optocoupler->getSense().right;
    Cell cell_from_sensors = s->optocoupler->getRelativeCell();

    // регулятор на положение по горизонтали при движении вперёд
    uint8_t regulatorState = toBool(cell_from_sensors.west_wall) << 1 | toBool(cell_from_sensors.east_wall);

    const int32_t LEFT_TRASHHOLD = s->optocoupler->SENSE_THRESHOLD_LEFT;
    const int32_t RIGHT_TRASHHOLD = s->optocoupler->SENSE_THRESHOLD_RIGHT;

    const float regulatorArray[4] = {
        0,//ни один не видит стену
        ANGLLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * (right_sense - RIGHT_TRASHHOLD - OPTOCOUPLER_SENSE_ERROR),//стену видит только правый
        ANGLLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * (LEFT_TRASHHOLD + OPTOCOUPLER_SENSE_ERROR - left_sense),//стену видит только левый
        ANGLLE_SPEED_OPTOCOUPLER_TWOSEN_REG_K * (right_sense - left_sense),//оба датчика  
    };

    ms->theta_i0 = regulatorArray[regulatorState];

    if(s->robotState->getDist() > CELL_SIZE)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(DIA)
{
    
}


//search turns 90
CYCLOGRAM(SS90EL)
{
    
    constexpr float R = SEARCH_TURN_RADIUS; //радиус поворота
    ms->v_f0 = FORWARD_SPEED;
    constexpr float theta_i = FORWARD_SPEED / R;

    constexpr float forwDist = CELL_SIZE / 2 - R;
    constexpr float circleDist = (2 * PI * R) / 4; // 90 - четверть окружности #КОСТЫЛЬ

    //if(s->robotState->getDist() > forwDist && s->robotState->getTheta() < HALF_PI) ms->theta_i0 = theta_i;
    if(s->robotState->getDist() > forwDist && s->robotState->getDist() < forwDist + circleDist) ms->theta_i0 = theta_i;
    else ms->theta_i0 = 0;
    if(s->robotState->getDist() > 2 * forwDist + circleDist)
    {
        ms->isComplete = true;
    } 
}

CYCLOGRAM(SS90ER)
{
    constexpr float R = SEARCH_TURN_RADIUS; //радиус поворота
    ms->v_f0 = FORWARD_SPEED;
    constexpr float theta_i = FORWARD_SPEED / R;

    constexpr float forwDist = CELL_SIZE / 2 - R;
    constexpr float circleDist = (2 * PI * R) / 4; // 90 - четверть окружности #КОСТЫЛЬ

    //if(s->robotState->getDist() > forwDist && s->robotState->getTheta() < HALF_PI) ms->theta_i0 = theta_i;
    if(s->robotState->getDist() > forwDist && s->robotState->getDist() < forwDist + circleDist) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;
    if(s->robotState->getDist() > 2 * forwDist + circleDist)
    {
        ms->isComplete = true;
    } 
}

CYCLOGRAM(SS90SL)
{
    constexpr float R = SS90S_TURN_RADIUS; //радиус поворота
    ms->v_f0 = FORWARD_SPEED;
    constexpr float theta_i = FORWARD_SPEED / R;

    constexpr float forwDist = CELL_SIZE * 1.5 / 2 - R; 
    constexpr float circleDis = (2 * PI * R) / 4; // 90 = четверть окружности
    
    //if(s->robotState->getDist() > forwDist && s->robotState->getTheta() < HALF_PI) ms->theta_i0 = -theta_i;
    if(s->robotState->getDist() > forwDist && s->robotState->getDist() < forwDist + circleDis) ms->theta_i0 = theta_i;
    else ms->theta_i0 = 0;
    if(s->robotState->getDist() > 2 * forwDist + circleDis) 
    {
        ms->isComplete = true;
    }
}

CYCLOGRAM(SS90SL)
{
    constexpr float R = SS90S_TURN_RADIUS; //радиус поворота
    ms->v_f0 = FORWARD_SPEED;
    constexpr float theta_i = FORWARD_SPEED / R;

    constexpr float forwDist = CELL_SIZE * 1.5 / 2 - R; 
    constexpr float circleDis = (2 * PI * R) / 4; // 90 = четверть окружности
    
    //if(s->robotState->getDist() > forwDist && s->robotState->getTheta() < HALF_PI) ms->theta_i0 = -theta_i;
    if(s->robotState->getDist() > forwDist && s->robotState->getDist() < forwDist + circleDis) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;
    if(s->robotState->getDist() > 2 * forwDist + circleDis) 
    {
        ms->isComplete = true;
    }
}


CYCLOGRAM(SD45SL)
{
    constexpr float forwDist = SD45S_FORW_DIST; // путь до начала поворота, максимум - CELL_SIZE / 2
    constexpr float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    constexpr float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 - (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDis = (2 * PI * R) / 8; // 45 = 1/8 окружности
    ms->v_f0 = FORWARD_SPEED;
    float theta_i = FORWARD_SPEED / R;

    if(s->robotState->getDist() > forwDist && s->robotState->getDist() < forwDist + circleDis) ms->theta_i0 = theta_i;
    else ms->theta_i0 = 0;

    if(s->robotState->getDist() > forwDist + circleDis + forwDist2) ms->isComplete = true;
}

CYCLOGRAM(SD45SR)
{
    constexpr float forwDist = SD45S_FORW_DIST; // путь до начала поворота, максимум - CELL_SIZE / 2
    constexpr float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    constexpr float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 - (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDis = (2 * PI * R) / 8; // 45 = 1/8 окружности
    ms->v_f0 = FORWARD_SPEED;
    float theta_i = FORWARD_SPEED / R;

    if(s->robotState->getDist() > forwDist && s->robotState->getDist() < forwDist + circleDis) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;

    if(s->robotState->getDist() > forwDist + circleDis + forwDist2) ms->isComplete = true;
}

CYCLOGRAM(DS45SL)
{
    constexpr float forwDist = SD45S_FORW_DIST; // путь до начала поворота, максимум - CELL_SIZE / 2
    constexpr float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    constexpr float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 - (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDis = (2 * PI * R) / 8; // 45 = 1/8 окружности
    ms->v_f0 = FORWARD_SPEED;
    float theta_i = FORWARD_SPEED / R;

    if(s->robotState->getDist() > forwDist2 && s->robotState->getDist() < forwDist2 + circleDis) ms->theta_i0 = theta_i;
    else ms->theta_i0 = 0;

    if(s->robotState->getDist() > forwDist + circleDis + forwDist2) ms->isComplete = true;
}

CYCLOGRAM(DS45SR)
{
    constexpr float forwDist = SD45S_FORW_DIST; // путь до начала поворота, максимум - CELL_SIZE / 2
    constexpr float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    constexpr float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 - (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDis = (2 * PI * R) / 8; // 45 = 1/8 окружности
    ms->v_f0 = FORWARD_SPEED;
    float theta_i = FORWARD_SPEED / R;

    if(s->robotState->getDist() > forwDist2 && s->robotState->getDist() < forwDist2 + circleDis) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;

    if(s->robotState->getDist() > forwDist + circleDis + forwDist2) ms->isComplete = true;
}

CYCLOGRAM(SD135SL)
{
    constexpr float R = SD135S_TURN_RADIUS; //радиус поворота
    ms->v_f0 = FORWARD_SPEED;
    constexpr float theta_i = FORWARD_SPEED / R;

    constexpr float forwDist1 = 1.5 * CELL_SIZE - R * (1 + M_SQRT2);
    constexpr float forwDist2 = R + M_SQRT2 * CELL_SIZE - R * (M_SQRT2 + 2);
    constexpr float circleDis = (2 * PI * R) * (135.0 / 360); //доля длины окружности в 135 градусах

    if(s->robotState->getDist() > forwDist1 && s->robotState->getDist() < forwDist1 + circleDis) ms->theta_i0 = theta_i;
    else ms->theta_i0 = 0;
    if(s->robotState->getDist() > forwDist1 + circleDis + forwDist2) ms->isComplete = true;
}

CYCLOGRAM(SD135SR)
{
    constexpr float R = SD135S_TURN_RADIUS; //радиус поворота
    ms->v_f0 = FORWARD_SPEED;
    constexpr float theta_i = FORWARD_SPEED / R;

    constexpr float forwDist1 = 1.5 * CELL_SIZE - R * (1 + M_SQRT2);
    constexpr float forwDist2 = R + M_SQRT2 * CELL_SIZE - R * (M_SQRT2 + 2);
    constexpr float circleDis = (2 * PI * R) * (135.0 / 360); //доля длины окружности в 135 градусах

    if(s->robotState->getDist() > forwDist1 && s->robotState->getDist() < forwDist1 + circleDis) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;
    if(s->robotState->getDist() > forwDist1 + circleDis + forwDist2) ms->isComplete = true;
}

CYCLOGRAM(DS135SL)
{
    constexpr float R = SD135S_TURN_RADIUS; //радиус поворота
    ms->v_f0 = FORWARD_SPEED;
    constexpr float theta_i = FORWARD_SPEED / R;

    constexpr float forwDist1 = 1.5 * CELL_SIZE - R * (1 + M_SQRT2);
    constexpr float forwDist2 = R + M_SQRT2 * CELL_SIZE - R * (M_SQRT2 + 2);
    constexpr float circleDis = (2 * PI * R) * (135.0 / 360); //доля длины окружности в 135 градусах

    if(s->robotState->getDist() > forwDist2 && s->robotState->getDist() < forwDist2 + circleDis) ms->theta_i0 = theta_i;
    else ms->theta_i0 = 0;
    if(s->robotState->getDist() > forwDist1 + circleDis + forwDist2) ms->isComplete = true;
}

CYCLOGRAM(DS135SR)
{
    constexpr float R = SD135S_TURN_RADIUS; //радиус поворота
    ms->v_f0 = FORWARD_SPEED;
    constexpr float theta_i = FORWARD_SPEED / R;

    constexpr float forwDist1 = 1.5 * CELL_SIZE - R * (1 + M_SQRT2);
    constexpr float forwDist2 = R + M_SQRT2 * CELL_SIZE - R * (M_SQRT2 + 2);
    constexpr float circleDis = (2 * PI * R) * (135.0 / 360); //доля длины окружности в 135 градусах

    if(s->robotState->getDist() > forwDist2 && s->robotState->getDist() < forwDist2 + circleDis) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;
    if(s->robotState->getDist() > forwDist1 + circleDis + forwDist2) ms->isComplete = true;
}

CYCLOGRAM(SS180L){}
CYCLOGRAM(SS180R){}

CYCLOGRAM(IP180)
{
    ms->v_f0 = 0;
    constexpr float theta_i = FORWARD_SPEED / ROBOT_WIDTH / 2;
    ms->theta_i0 = theta_i;
    if(s->robotState->getTheta() > PI) ms->isComplete = true;
}

CYCLOGRAM(IP90L)
{
    ms->v_f0 = 0;
    constexpr float theta_i = FORWARD_SPEED / ROBOT_WIDTH / 2;
    ms->theta_i0 = theta_i;
    if(s->robotState->getTheta() > HALF_PI) ms->isComplete = true;
}

CYCLOGRAM(IP90R)
{
    ms->v_f0 = 0;
    constexpr float theta_i = -FORWARD_SPEED / ROBOT_WIDTH / 2;
    ms->theta_i0 = theta_i;
    if(s->robotState->getTheta() < -HALF_PI) ms->isComplete = true;
}

#endif // !_CYCLOGRAMS_H_