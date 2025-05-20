#ifndef _CYCLOGRAMS_H_
#define _CYCLOGRAMS_H_

#include "CycloUtilits/CycloTypes.h"
#include "Cyclogram.config.h"

#include "smart45.h"
#include "smart135.h"
#include "alignment.h"
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

CYCLOGRAM(CLUSTER_DOT){
    ms->isComplete = true;
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

    const int16_t left_sense = s->optocoupler->getSense().left;
    const int16_t right_sense = s->optocoupler->getSense().right;
    const Cell cell_from_sensors = s->optocoupler->getRelativeCell();

    // регулятор на положение по горизонтали при движении вперёд
    const uint8_t regulatorState = toBool(cell_from_sensors.west_wall) << 1 | toBool(cell_from_sensors.east_wall);

    const int16_t LEFT_TRASHHOLD = s->optocoupler->SENSE_THRESHOLD_LEFT;
    const int16_t RIGHT_TRASHHOLD = s->optocoupler->SENSE_THRESHOLD_RIGHT;

    const float regulatorArray[4] = {
        0,//ни один не видит стену
        ANGLLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * (right_sense - RIGHT_TRASHHOLD - OPTOCOUPLER_SENSE_ERROR),//стену видит только правый
        ANGLLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * (LEFT_TRASHHOLD + OPTOCOUPLER_SENSE_ERROR - left_sense),//стену видит только левый
        ANGLLE_SPEED_OPTOCOUPLER_TWOSEN_REG_K * (right_sense - left_sense),//оба датчика
    };

    ms->theta_i0 = regulatorArray[regulatorState];

    if(s->odometry->getDist() > HALF(CELL_SIZE) )
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(FWDE)
{
    ms->v_f0 = FORWARD_SPEED;
    ms->theta_i0 = 0;

    static bool FIRST_ENTRANCE = 1;
    static bool start_left_wall_state = 0;
    static bool start_right_wall_state = 0;
    static bool prev_left_wall_state = 0;
    static bool prev_right_wall_state = 0;
    static float dist_buf = 0;
    static bool CAN_TRY_ALIGN = 0;
    static bool NEED_ALIGN = 0;

    const int16_t left_sense = s->optocoupler->getSense().left;
    const int16_t right_sense = s->optocoupler->getSense().right;
    const Cell cell_from_sensors = s->optocoupler->getRelativeCell();

    // регулятор на положение по горизонтали при движении вперёд
    const uint8_t regulatorState = toBool(cell_from_sensors.west_wall) << 1 | toBool(cell_from_sensors.east_wall);

    const int16_t LEFT_TRASHHOLD = s->optocoupler->SENSE_THRESHOLD_LEFT;
    const int16_t RIGHT_TRASHHOLD = s->optocoupler->SENSE_THRESHOLD_RIGHT;

    const float regulatorArray[4] = {
        0,//ни один не видит стену
        ANGLLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * (right_sense - RIGHT_TRASHHOLD - OPTOCOUPLER_SENSE_ERROR),//стену видит только правый
        ANGLLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * (LEFT_TRASHHOLD + OPTOCOUPLER_SENSE_ERROR - left_sense),//стену видит только левый
        ANGLLE_SPEED_OPTOCOUPLER_TWOSEN_REG_K * (right_sense - left_sense),//оба датчика
    };

    if(FIRST_ENTRANCE){
        start_left_wall_state = toBool(cell_from_sensors.west_wall);
        start_right_wall_state = toBool(cell_from_sensors.east_wall);

        CAN_TRY_ALIGN = start_left_wall_state || start_right_wall_state;
        FIRST_ENTRANCE = 0;
    }

    ms->theta_i0 = regulatorArray[regulatorState];

    if(CAN_TRY_ALIGN && ((
        prev_left_wall_state == 1 && toBool(cell_from_sensors.west_wall) == 0) || (
       prev_right_wall_state == 1 && toBool(cell_from_sensors.east_wall) == 0))){
            NEED_ALIGN = 1;
            dist_buf = s->odometry->getDist();
    }

    prev_left_wall_state = toBool(cell_from_sensors.west_wall);
    prev_right_wall_state = toBool(cell_from_sensors.east_wall);

    if(NEED_ALIGN)
    {
        if(s->odometry->getDist() - dist_buf > FROM_ZERO_WALL_TO_SIDE){
            ms->isComplete = true;
        }
    }
    else if(s->odometry->getDist() > CELL_SIZE)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;

    if(ms->isComplete){
        FIRST_ENTRANCE = 1;
        prev_left_wall_state = 0;
        prev_right_wall_state = 0;
        CAN_TRY_ALIGN = 0;
        NEED_ALIGN = 0;
    }
}

CYCLOGRAM(FWD_X)
{
    ms->v_f0 = FORWARD_SPEED;
    ms->theta_i0 = 0;

    const int16_t left_sense = s->optocoupler->getSense().left;
    const int16_t right_sense = s->optocoupler->getSense().right;
    const Cell cell_from_sensors = s->optocoupler->getRelativeCell();

    // регулятор на положение по горизонтали при движении вперёд
    const uint8_t regulatorState = toBool(cell_from_sensors.west_wall) << 1 | toBool(cell_from_sensors.east_wall);

    const int16_t LEFT_TRASHHOLD = s->optocoupler->SENSE_THRESHOLD_LEFT;
    const int16_t RIGHT_TRASHHOLD = s->optocoupler->SENSE_THRESHOLD_RIGHT;

    const float regulatorArray[4] = {
        0,//ни один не видит стену
        ANGLLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * (right_sense - RIGHT_TRASHHOLD - OPTOCOUPLER_SENSE_ERROR),//стену видит только правый
        ANGLLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * (LEFT_TRASHHOLD + OPTOCOUPLER_SENSE_ERROR - left_sense),//стену видит только левый
        ANGLLE_SPEED_OPTOCOUPLER_TWOSEN_REG_K * (right_sense - left_sense),//оба датчика
    };

    ms->theta_i0 = regulatorArray[regulatorState];

    if(s->odometry->getDist() > CELL_SIZE * x)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(DIAG_X)
{
    ms->v_f0 = FORWARD_SPEED;
    ms->theta_i0 = 0;

    if(s->odometry->getDist() > CELL_SIZE / M_SQRT2 * x)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

//search turns 90
CYCLOGRAM(SS90EL)
{
    ms->v_f0 = FORWARD_SPEED;
    constexpr float R = SEARCH_TURN_RADIUS; //радиус поворота
    constexpr float theta_i = FORWARD_SPEED / R;

    constexpr float forwDist = CELL_SIZE / 2 - R;
    constexpr float circleDist = (2 * PI * R) / 4;
    
    if(s->odometry->getDist() > forwDist && s->odometry->getDist() < forwDist + circleDist){
        ms->theta_i0 = theta_i;
    }
    else ms->theta_i0 = 0;
    if(s->odometry->getDist() > 2 * forwDist + circleDist)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(SS90ER)
{
    ms->v_f0 = FORWARD_SPEED;
    constexpr float R = SEARCH_TURN_RADIUS; //радиус поворота
    constexpr float theta_i = FORWARD_SPEED / R;

    constexpr float forwDist = CELL_SIZE / 2 - R;
    constexpr float circleDist = (2 * PI * R) / 4;
    
    if(s->odometry->getDist() > forwDist && s->odometry->getDist() < forwDist + circleDist){
        ms->theta_i0 = -theta_i;
    }
    else ms->theta_i0 = 0;
    if(s->odometry->getDist() > 2 * forwDist + circleDist)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;

}

CYCLOGRAM(SS90SL)
{
    ms->v_f0 = FORWARD_SPEED;
    constexpr float R = SS90S_TURN_RADIUS; //радиус поворота
    constexpr float theta_i = FORWARD_SPEED / R;

    constexpr float forwDist = CELL_SIZE * 1.5 / 2 - R;
    constexpr float circleDist = (2 * PI * R) / 4;

    if(s->odometry->getDist() > forwDist && s->odometry->getDist() < forwDist + circleDist) ms->theta_i0 = theta_i;
    else ms->theta_i0 = 0;
    if(s->odometry->getDist() > 2 * forwDist + circleDist)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(SS90SR)
{
    ms->v_f0 = FORWARD_SPEED;
    constexpr float R = SS90S_TURN_RADIUS; //радиус поворота
    constexpr float theta_i = FORWARD_SPEED / R;

    constexpr float forwDist = CELL_SIZE * 1.5 / 2 - R;
    constexpr float circleDist = (2 * PI * R) / 4;

    if(s->odometry->getDist() > forwDist && s->odometry->getDist() < forwDist + circleDist) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;
    if(s->odometry->getDist() > 2 * forwDist + circleDist)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}
CYCLOGRAM(DD90SL)
{
    ms->v_f0 = FORWARD_SPEED;
    constexpr float R = DD90S_TURN_RADIUS; //радиус поворота
    constexpr float forwDist = CELL_SIZE / M_SQRT2 - R + 0.005;
    constexpr float circleDist = (2 * PI * R) / 4; // 90 = четверть окружности
    float theta_i = FORWARD_SPEED / R;

    if(s->odometry->getDist() > forwDist && s->odometry->getDist() < forwDist + circleDist) ms->theta_i0 = theta_i;
    else ms->theta_i0 = 0;

    if(s->odometry->getDist() > forwDist + circleDist + forwDist)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}
CYCLOGRAM(DD90SR)
{
    ms->v_f0 = FORWARD_SPEED;
    constexpr float R = DD90S_TURN_RADIUS; //радиус поворота
    constexpr float forwDist = CELL_SIZE / M_SQRT2 - R;
    constexpr float circleDist = (2 * PI * R) / 4; // 90 = четверть окружности
    float theta_i = FORWARD_SPEED / R;

    if(s->odometry->getDist() > forwDist && s->odometry->getDist() < forwDist + circleDist) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;

    if(s->odometry->getDist() > forwDist + circleDist + forwDist)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}


CYCLOGRAM(SS180SL)
{
    ms->v_f0 = FORWARD_SPEED;
    constexpr float R = CELL_SIZE / 2;
    constexpr float theta_i = FORWARD_SPEED / R;
    constexpr float circleDist = PI * R; // 180 = половина окружности
    constexpr float forwDist = SS180S_FORW_DIST;

    if(s->odometry->getDist() > forwDist && s->odometry->getDist() < forwDist + circleDist) ms->theta_i0 = theta_i;
    else ms->theta_i0 = 0;
    if(s->odometry->getDist() > 2 * forwDist + circleDist)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}
CYCLOGRAM(SS180SR)
{
    ms->v_f0 = FORWARD_SPEED;
    constexpr float R = CELL_SIZE / 2;
    constexpr float theta_i = FORWARD_SPEED / R;
    constexpr float circleDist = PI * R; // 180 = половина окружности
    constexpr float forwDist = SS180S_FORW_DIST;

    if(s->odometry->getDist() > forwDist && s->odometry->getDist() < forwDist + circleDist) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;
    if(s->odometry->getDist() > 2 * forwDist + circleDist)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(IP180)
{
    ms->v_f0 = 0;
    constexpr float theta_i = FORWARD_SPEED / (ROBOT_WIDTH / 2);
    ms->theta_i0 = theta_i;
    if(s->odometry->getTheta() >= PI)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(IP90L)
{
    constexpr float theta_i = FORWARD_SPEED / (ROBOT_WIDTH / 2);
    constexpr float THETA_ERROR = 6 * PI / 180;

    ms->v_f0 = 0;
    ms->theta_i0 = theta_i;
    
    if(s->odometry->getTheta() >= HALF_PI - THETA_ERROR)
    {
        ms->theta_i0 = 0;
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(IP90R)
{
    constexpr float theta_i = -FORWARD_SPEED / (ROBOT_WIDTH / 2);
    constexpr float THETA_ERROR = 6 * PI / 180;

    ms->v_f0 = 0;
    ms->theta_i0 = theta_i;

    if(s->odometry->getTheta() <= -HALF_PI + THETA_ERROR)
    {
        ms->theta_i0 = 0;
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

#endif // !_CYCLOGRAMS_H_