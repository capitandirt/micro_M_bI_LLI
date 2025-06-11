#include "../CycloTypes.h"
#include "Cyclogram.config.h"

inline float getThetaIFromAngleReg(const Sensors* s, const float THETA_0)
{
    const float cur_theta = s->odometry->getTheta();
    
    float theta_err = circle_mod(THETA_0 - cur_theta);
    return theta_err * ANGLE_REG_KP;
}

inline float getThetaIFromWallReg(const Sensors* s) // не используется
{
    const int16_t left_sense = s->optocoupler->getSense().left;
    const int16_t right_sense = s->optocoupler->getSense().right;
    const Cell cell_from_sensors = s->optocoupler->getRelativeCell();

    // регулятор на положение по горизонтали при движении вперёд
    const uint8_t regulatorState = toBool(cell_from_sensors.west_wall) << 1 | toBool(cell_from_sensors.east_wall);

    const float regulatorErr[4] = 
    {
        0,
        (right_sense - s->optocoupler->getRightSense0()),
        (s->optocoupler->getLeftSense0() - left_sense),
        (right_sense - left_sense - s->optocoupler->getStaticError())
    };

    const float regulatorArray[4] = {
        0,//ни один не видит стену
        ANGLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * regulatorErr[1],//стену видит только правый
        ANGLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * regulatorErr[2],//стену видит только левый
        ANGLE_SPEED_OPTOCOUPLER_TWOSEN_REG_K * regulatorErr[3],//оба датчика
    };
    return regulatorArray[regulatorState];
}


inline void FWD_default(MotionStates* ms, const Sensors* s, const float THETA_0)
{
    const int16_t left_sense = s->optocoupler->getSense().left;
    const int16_t right_sense = s->optocoupler->getSense().right;
    const Cell cell_from_sensors = s->optocoupler->getRelativeCell();

    // регулятор на положение по горизонтали при движении вперёд
    const uint8_t regulatorState = toBool(cell_from_sensors.west_wall) << 1 | toBool(cell_from_sensors.east_wall);

    const float regulatorErr[4] = 
    {
        0,
        (right_sense - s->optocoupler->getRightSense0()),
        (s->optocoupler->getLeftSense0() - left_sense),
        (right_sense - left_sense - s->optocoupler->getStaticError())
    };

    const float regulatorArray[4] = {
        0,//ни один не видит стену
        ANGLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * regulatorErr[1],//стену видит только правый
        ANGLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * regulatorErr[2],//стену видит только левый
        ANGLE_SPEED_OPTOCOUPLER_TWOSEN_REG_K * regulatorErr[3],//оба датчика
    };

    const float err = regulatorErr[regulatorState];
    
    ms->theta_i0 = 0;
    if(regulatorState == 0 || abs(err) < ANGLE_REG_THRESHOLD) 
    {
        #if USE_ANGLE_REGULATOR
        ms->theta_i0 = getThetaIFromAngleReg(s, THETA_0);
        #else
        ms->theta_i0 = 0;
        #endif
    }
    else ms->theta_i0 += regulatorArray[regulatorState];
}