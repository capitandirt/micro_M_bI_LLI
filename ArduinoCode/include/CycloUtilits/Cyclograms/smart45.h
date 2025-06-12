#ifndef _SMART45_H_
#define _SMART45_H_

#include "CycloUtilits/CycloTypes.h"
#include "Cyclogram.config.h"

CYCLOGRAM(SD45SL)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float forwDist = SD45S_FORW_DIST; // путь до начала поворота, максимум - CELL_SIZE / 2
    constexpr float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    constexpr float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 - (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDist = (2 * PI * R) / 8; // 45 = 1/8 окружности
    float theta_i = FAST_FORWARD_SPEED / R;

    // Serial.println(s->odometry->getRelativeTheta() * RAD_TO_DEG);
    if(s->odometry->getRelativeDist() < forwDist)
    {
        FWD_default(ms, s, ms->theta_0);
    }    
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() < HALF(HALF_PI)) ms->theta_i0 = theta_i;
    #else
    else if(s->odometry->getRelativeDist() > forwDist && s->odometry->getRelativeDist() < forwDist + circleDist) ms->theta_i0 = theta_i;
    #endif
    else ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0 + HALF(HALF_PI));

    if(s->odometry->getRelativeDist() > forwDist + circleDist + forwDist2)
    { 
        ms->isComplete = true;
        ms->theta_0 += HALF(HALF_PI);
    }
    else ms->isComplete = false;
}

CYCLOGRAM(SD45SR)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float forwDist = SD45S_FORW_DIST; // путь до начала поворота, максимум - CELL_SIZE / 2
    constexpr float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    constexpr float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 - (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDist = (2 * PI * R) / 8; // 45 = 1/8 окружности
    float theta_i = FAST_FORWARD_SPEED / R;

    if(s->odometry->getRelativeDist() < forwDist)
    {
        FWD_default(ms, s, ms->theta_0);
    }
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() > -HALF(HALF_PI)) ms->theta_i0 = -theta_i;
    #else
    else if(s->odometry->getRelativeDist() > forwDist && s->odometry->getRelativeDist() < forwDist + circleDist) ms->theta_i0 = -theta_i;
    #endif
    else ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0 - HALF(HALF_PI));

    if(s->odometry->getRelativeDist() > forwDist + circleDist + forwDist2)
    { 
        ms->isComplete = true;
        ms->theta_0 -= HALF(HALF_PI);
    }
    else ms->isComplete = false;
}

CYCLOGRAM(DS45SL)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float forwDist = SD45S_FORW_DIST; // путь до начала поворота, максимум - CELL_SIZE / 2
    constexpr float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    constexpr float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 - (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDist = (2 * PI * R) / 8; // 45 = 1/8 окружности
    float theta_i = FAST_FORWARD_SPEED / R;


    // Serial.println(s->odometry->getRelativeTheta() * RAD_TO_DEG);
    if(s->odometry->getRelativeDist() < forwDist2)
    {
        ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0);
    }
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() < HALF(HALF_PI)) ms->theta_i0 = theta_i;
    #else
    else if(s->odometry->getRelativeDist() > forwDist2 && s->odometry->getRelativeDist() < forwDist2 + circleDist) ms->theta_i0 = theta_i;
    #endif
    else
    {
        FWD_default(ms, s, ms->theta_0 + HALF(HALF_PI));
    }

    if(s->odometry->getRelativeDist() > forwDist + circleDist + forwDist2)
    { 
        ms->isComplete = true;
        ms->theta_0 += HALF(HALF_PI);
    }
    else ms->isComplete = false;
}

CYCLOGRAM(DS45SR)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float forwDist = SD45S_FORW_DIST; // путь до начала поворота, максимум - CELL_SIZE / 2
    constexpr float forwDist2 = M_SQRT2 * CELL_SIZE / 2 - (CELL_SIZE / 2 - forwDist); //путь после конца поворота
    constexpr float R = (CELL_SIZE / 2 - forwDist) * (1 + M_SQRT2); //радиус поворота, максимум - 0.21 - (CELL_SIZE / 2 * (1 + sqrt(2)))

    const float circleDist = (2 * PI * R) / 8; // 45 = 1/8 окружности
    float theta_i = FAST_FORWARD_SPEED / R;

    if(s->odometry->getRelativeDist() < forwDist2)
    {
        ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0);
    }
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() > -HALF(HALF_PI)) ms->theta_i0 = -theta_i;
    #else
    else if(s->odometry->getRelativeDist() > forwDist2 && s->odometry->getRelativeDist() < forwDist2 + circleDist) ms->theta_i0 = -theta_i;
    #endif
    else
    {
        ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0 - HALF(HALF_PI));
        //FWD_default(ms, s, ms->theta_0 - HALF(HALF_PI));
    }

    if(s->odometry->getRelativeDist() > forwDist + circleDist + forwDist2)
    { 
        ms->isComplete = true;
        ms->theta_0 -= HALF(HALF_PI);
    }
    else ms->isComplete = false;
}

#endif // !_SMART45_H_