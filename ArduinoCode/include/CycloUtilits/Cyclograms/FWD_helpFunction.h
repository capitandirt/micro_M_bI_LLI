#ifndef _FWD_HELPFUNCTION_H_
#define _FWD_HELPFUNCTION_H_

#include "../CycloTypes.h"
#include "Cyclogram.config.h"


inline void FWD_helpFunction(MotionStates* ms, const Sensors* s, const float THETA_0, bool isSmartSpeed = false)
{
    ms->v_f0 = FORWARD_SPEED;
    if(isSmartSpeed) ms->v_f0 = SMART_FORWARD_SPEED;
    //ms->theta_i0 = 0;
    float theta_err = THETA_0 - s->odometry->getTheta();
    ms->theta_i0 = 0;//theta_err * ANGLE_REG_KP;

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

    ms->theta_i0 += regulatorArray[regulatorState];
    //Serial.println(theta_err * ANGLE_REG_KP);
}

#endif // !_FWD_HELPFUNCTION_H_