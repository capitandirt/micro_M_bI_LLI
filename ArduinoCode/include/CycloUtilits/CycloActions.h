#ifndef _CYCLO_ACTIONS_H
#define _CYCLO_ACTIONS_H

#include "Config.h"
#include "CycloTypes.h"
/* РАСШИФРОВКА НАЗВАНИЙ ПОВОРОТОВ
SS90EL
S - from straight (0 / 90... градусов поворот) или D - from diagonal
S - to straight или to diagonal
90 - градус поворота
E - explorer или S - smooth
L - левый или R - правый
*/
//исключение - IP90 - In Place

typedef void (*Cyclogram)(MotionStates*, Sensors);
#define CYCLOGRAM(name) inline void name(MotionStates* ms, Sensors* s)

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

CYCLOGRAM(FWD)
{
    ms->v_f0 = FORWARD_SPEED;
    ms->theta_i0 = 0;
    
    if(s->robotState->getDist() > CELL_SIZE)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}
CYCLOGRAM(FWD_HALF)
{
    ms->v_f0 = FORWARD_SPEED;
    ms->theta_i0 = 0;
    
    if(s->robotState->getDist() > CELL_SIZE / 2)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(SS90SL){}; //ДОПИСАТЬ
CYCLOGRAM(SS90SR){};

//search turns 90
CYCLOGRAM(SS90EL)
{
    const float R = 0.07; //радиус поворота
    ms->v_f0 = FORWARD_SPEED;
    float theta_i = FORWARD_SPEED / R;

    float forwDist = CELL_SIZE / 2 - R;
    float circleDis = (2 * PI * R) / 4; // 90 - четверть окружности #КОСТЫЛЬ

    if(s->robotState->getDist() > forwDist && s->robotState->getDist() < forwDist + circleDis) ms->theta_i0 = theta_i;
    else ms->theta_i0 = 0;
    if(s->robotState->getDist() > 2 * forwDist + circleDis)
    {
        ms->isComplete = true;
    } 
}
CYCLOGRAM(SS90ER)
{
    const float R = 0.07; //радиус поворота
    ms->v_f0 = FORWARD_SPEED;
    float theta_i = FORWARD_SPEED / R;

    const float forwDist = CELL_SIZE / 2 - R;
    const float circleDis = (2 * PI * R) / 4; // 90 = четверть окружности
    if(s->robotState->getDist() > forwDist && s->robotState->getDist() < forwDist + circleDis) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;
    if(s->robotState->getDist() > 2 * forwDist + circleDis) 
    {
        ms->isComplete = true;
    }
}

//45 turns
CYCLOGRAM(SD45SL)
{
    const float forwDist = CELL_SIZE / 4; // путь до начала поворота, максимум - CELL_SIZE / 2
    const float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    const float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDis = (2 * PI * R) / 8; // 45 = 1/8 окружности
    ms->v_f0 = FORWARD_SPEED;
    float theta_i = FORWARD_SPEED / R;

    if(s->robotState->getDist() > forwDist && s->robotState->getDist() < forwDist + circleDis) ms->theta_i0 = theta_i;
    else ms->theta_i0 = 0;

    if(s->robotState->getDist() > forwDist + circleDis + forwDist2) ms->isComplete = true;
}
CYCLOGRAM(SD45SR)
{
    const float forwDist = CELL_SIZE / 4; // путь до начала поворота, максимум - CELL_SIZE / 2
    const float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    const float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDis = (2 * PI * R) / 8; // 45 = 1/8 окружности
    ms->v_f0 = FORWARD_SPEED;
    float theta_i = FORWARD_SPEED / R;

    if(s->robotState->getDist() > forwDist && s->robotState->getDist() < forwDist + circleDis) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;

    if(s->robotState->getDist() > forwDist + circleDis + forwDist2) ms->isComplete = true;
}
CYCLOGRAM(DS45SL)
{
    const float forwDist = CELL_SIZE / 4; // путь до начала поворота, максимум - CELL_SIZE / 2
    const float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    const float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDis = (2 * PI * R) / 8; // 45 = 1/8 окружности
    ms->v_f0 = FORWARD_SPEED;
    float theta_i = FORWARD_SPEED / R;

    if(s->robotState->getDist() > forwDist2 && s->robotState->getDist() < forwDist2 + circleDis) ms->theta_i0 = theta_i;
    else ms->theta_i0 = 0;

    if(s->robotState->getDist() > forwDist + circleDis + forwDist2) ms->isComplete = true;
}
CYCLOGRAM(DS45SR)
{
    const float forwDist = CELL_SIZE / 4; // путь до начала поворота, максимум - CELL_SIZE / 2
    const float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    const float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDis = (2 * PI * R) / 8; // 45 = 1/8 окружности
    ms->v_f0 = FORWARD_SPEED;
    float theta_i = FORWARD_SPEED / R;

    if(s->robotState->getDist() > forwDist2 && s->robotState->getDist() < forwDist2 + circleDis) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;

    if(s->robotState->getDist() > forwDist + circleDis + forwDist2) ms->isComplete = true;
}



CYCLOGRAM(SD135SR)
{
    const float R = 0.07; //радиус поворота
    ms->v_f0 = FORWARD_SPEED;
    float theta_i = FORWARD_SPEED / R;

    const float forwDist1 = 1.5 * CELL_SIZE - R * (1 + M_SQRT2);
    const float forwDist2 = R + M_SQRT2 * CELL_SIZE - R * (M_SQRT2 + 2);
    const float circleDis = (2 * PI * R) * (135.0 / 360); //доля длины окружности в 135 градусах

    if(s->robotState->getDist() > forwDist1 && s->robotState->getDist() < forwDist1 + circleDis) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;
    if(s->robotState->getDist() > forwDist1 + circleDis + forwDist2) ms->isComplete = true;
}
CYCLOGRAM(SD135SL)
{
    const float R = 0.07; //радиус поворота
    ms->v_f0 = FORWARD_SPEED;
    float theta_i = -FORWARD_SPEED / R;

    const float forwDist1 = 1.5 * CELL_SIZE - R * (1 + M_SQRT2);
    const float forwDist2 = R + M_SQRT2 * CELL_SIZE - R * (M_SQRT2 + 2);
    const float circleDis = (2 * PI * R) * (135.0 / 360); //доля длины окружности в 135 градусах

    if(s->robotState->getDist() > forwDist1 && s->robotState->getDist() < forwDist1 + circleDis) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;
    if(s->robotState->getDist() > forwDist1 + circleDis + forwDist2) ms->isComplete = true;
}


CYCLOGRAM(SS180S)
{
    ms->v_f0 = 0;
    float theta_i = FORWARD_SPEED / ROBOT_WIDTH / 2;
    ms->theta_i0 = theta_i;
    if(s->robotState->getTheta() > PI) ms->isComplete = true;
}
CYCLOGRAM(IP90L)
{
    ms->v_f0 = 0;
    float theta_i = FORWARD_SPEED / ROBOT_WIDTH / 2;
    ms->theta_i0 = theta_i;
    if(s->robotState->getTheta() > HALF_PI) ms->isComplete = true;
}
CYCLOGRAM(IP90R)
{
    ms->v_f0 = 0;
    float theta_i = -FORWARD_SPEED / ROBOT_WIDTH / 2;
    ms->theta_i0 = theta_i;
    if(s->robotState->getTheta() < -HALF_PI) ms->isComplete = true;
}

#endif // !_CYCLO_ACTIONS_H