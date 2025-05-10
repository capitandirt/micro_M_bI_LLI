#ifndef _SMART45_H_
#define _SMART45_H_

#include "../CycloTypes.h"
#include "../../OptocouplerSensors.h"
#include "Cyclogram.config.h"

CYCLOGRAM(SD45SL)
{
    ms->v_f0 = FORWARD_SPEED;
    constexpr float forwDist = SD45S_FORW_DIST; // путь до начала поворота, максимум - CELL_SIZE / 2
    constexpr float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    constexpr float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 - (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDis = (2 * PI * R) / 8; // 45 = 1/8 окружности
    float theta_i = FORWARD_SPEED / R;

    if(s->robotState->getDist() > forwDist && s->robotState->getDist() < forwDist + circleDis) ms->theta_i0 = theta_i;
    else ms->theta_i0 = 0;

    if(s->robotState->getDist() > forwDist + circleDis + forwDist2)
    { 
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(SD45SR)
{
    ms->v_f0 = FORWARD_SPEED;
    constexpr float forwDist = SD45S_FORW_DIST; // путь до начала поворота, максимум - CELL_SIZE / 2
    constexpr float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    constexpr float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 - (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDis = (2 * PI * R) / 8; // 45 = 1/8 окружности
    float theta_i = FORWARD_SPEED / R;

    if(s->robotState->getDist() > forwDist && s->robotState->getDist() < forwDist + circleDis) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;

    if(s->robotState->getDist() > forwDist + circleDis + forwDist2)
    { 
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(DS45SL)
{
    ms->v_f0 = FORWARD_SPEED;
    constexpr float forwDist = SD45S_FORW_DIST; // путь до начала поворота, максимум - CELL_SIZE / 2
    constexpr float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    constexpr float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 - (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDis = (2 * PI * R) / 8; // 45 = 1/8 окружности
    float theta_i = FORWARD_SPEED / R;

    if(s->robotState->getDist() > forwDist2 && s->robotState->getDist() < forwDist2 + circleDis) ms->theta_i0 = theta_i;
    else ms->theta_i0 = 0;

    if(s->robotState->getDist() > forwDist + circleDis + forwDist2)
    { 
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(DS45SR)
{
    ms->v_f0 = FORWARD_SPEED;
    constexpr float forwDist = SD45S_FORW_DIST; // путь до начала поворота, максимум - CELL_SIZE / 2
    constexpr float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    constexpr float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 - (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDis = (2 * PI * R) / 8; // 45 = 1/8 окружности
    float theta_i = FORWARD_SPEED / R;

    if(s->robotState->getDist() > forwDist2 && s->robotState->getDist() < forwDist2 + circleDis) ms->theta_i0 = -theta_i;
    else ms->theta_i0 = 0;

    if(s->robotState->getDist() > forwDist + circleDis + forwDist2)
    { 
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

#endif // !_SMART45_H_