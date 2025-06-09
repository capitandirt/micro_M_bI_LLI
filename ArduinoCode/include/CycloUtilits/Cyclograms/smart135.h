#ifndef _SMART135_H_
#define _SMART135_H_

#include "CycloUtilits/CycloTypes.h"
#include "Cyclogram.config.h"

CYCLOGRAM(SD135SL)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float R = SD135S_TURN_RADIUS; //радиус поворота
    constexpr float theta_i = FAST_FORWARD_SPEED / R;

    constexpr float forwDist1 = 1.5 * CELL_SIZE - R * (1 + M_SQRT2);
    constexpr float forwDist2 = R + M_SQRT2 * CELL_SIZE - R * (M_SQRT2 + 2);


    constexpr float circleDist = (2 * PI * R) * (135.0 / 360); //доля длины окружности в 135 градусах

    if(s->odometry->getRelativeDist() < forwDist1)
    {
        FWD_default(ms, s, ms->theta_0);
    }
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() < (HALF_PI + HALF(HALF_PI))) ms->theta_i0 = theta_i;
    #else
    else if(s->odometry->getRelativeDist() > forwDist1 && s->odometry->getRelativeDist() < forwDist1 + circleDist) ms->theta_i0 = theta_i;
    #endif
    else ms->theta_i0 = 0;
    if(s->odometry->getRelativeDist() > forwDist1 + circleDist + forwDist2)
    { 
        ms->isComplete = true;
        ms->theta_0 += (HALF_PI + HALF(HALF_PI));
    }
    else ms->isComplete = false;
}

CYCLOGRAM(SD135SR)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float R = SD135S_TURN_RADIUS; //радиус поворота
    constexpr float theta_i = FAST_FORWARD_SPEED / R;

    constexpr float forwDist1 = CELL_SIZE / 2 + (CELL_SIZE - R * (1 + M_SQRT2));
    constexpr float forwDist2 = CELL_SIZE * M_SQRT2 - R * (1 + M_SQRT2);
    constexpr float circleDist = (2 * PI * R) * (135.0 / 360); //доля длины окружности в 135 градусах

    if(s->odometry->getRelativeDist() < forwDist1)
    {
        FWD_default(ms, s, ms->theta_0);
    }
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() > -(HALF_PI + HALF(HALF_PI))) ms->theta_i0 = -theta_i;
    #else
    else if(s->odometry->getRelativeDist() > forwDist1 && s->odometry->getRelativeDist() < forwDist1 + circleDist) ms->theta_i0 = -theta_i;
    #endif
    else ms->theta_i0 = 0;
    if(s->odometry->getRelativeDist() > forwDist1 + circleDist + forwDist2)
    { 
        ms->isComplete = true;
        ms->theta_0 -= (HALF_PI + HALF(HALF_PI));
    }
    else ms->isComplete = false;
}

CYCLOGRAM(DS135SL)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float R = SD135S_TURN_RADIUS; //радиус поворота
    constexpr float theta_i = FAST_FORWARD_SPEED / R;

    constexpr float forwDist1 = 1.5 * CELL_SIZE - R * (1 + M_SQRT2);
    constexpr float forwDist2 = R + M_SQRT2 * CELL_SIZE - R * (M_SQRT2 + 2);
    constexpr float circleDist = (2 * PI * R) * (135.0 / 360); //доля длины окружности в 135 градусах

    if(s->odometry->getRelativeDist() < forwDist2)
    {
        ms->theta_i0 = 0;
    }
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() < (HALF_PI + HALF(HALF_PI))) ms->theta_i0 = theta_i;
    #else
    else if(s->odometry->getRelativeDist() > forwDist2 && s->odometry->getRelativeDist() < forwDist2 + circleDist) ms->theta_i0 = theta_i;
    #endif
    else
    {
        FWD_default(ms, s, ms->theta_0 + (HALF_PI + HALF(HALF_PI)));
    }
    if(s->odometry->getRelativeDist() > forwDist1 + circleDist + forwDist2)
    { 
        ms->isComplete = true;
        ms->theta_0 += (HALF_PI + HALF(HALF_PI));
    }
    else ms->isComplete = false;
}

CYCLOGRAM(DS135SR)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float R = SD135S_TURN_RADIUS; //радиус поворота
    constexpr float theta_i = FAST_FORWARD_SPEED / R;

    constexpr float forwDist1 = 1.5 * CELL_SIZE - R * (1 + M_SQRT2);
    constexpr float forwDist2 = R + M_SQRT2 * CELL_SIZE - R * (M_SQRT2 + 2);
    constexpr float circleDist = (2 * PI * R) * (135.0 / 360); //доля длины окружности в 135 градусах

    if(s->odometry->getRelativeDist() < forwDist2)
    {
        ms->theta_i0 = 0;
    }
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() > -(HALF_PI + HALF(HALF_PI))) ms->theta_i0 = -theta_i;
    #else
    else if(s->odometry->getRelativeDist() > forwDist2 && s->odometry->getRelativeDist() < forwDist2 + circleDist) ms->theta_i0 = -theta_i;
    #endif
    else
    {
        FWD_default(ms, s, ms->theta_0 - (HALF_PI + HALF(HALF_PI)));
    }
    if(s->odometry->getRelativeDist() > forwDist1 + circleDist + forwDist2)
    { 
        ms->isComplete = true;
        ms->theta_0 -= (HALF_PI + HALF(HALF_PI));
    }
    else ms->isComplete = false;
}

#endif // !_SMART135_H_