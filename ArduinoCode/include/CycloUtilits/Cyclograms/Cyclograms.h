#ifndef _CYCLOGRAMS_H_
#define _CYCLOGRAMS_H_

#include "CycloUtilits/CycloTypes.h"
#include "Cyclogram.config.h"

#include "FWD_helpFunction.h"
#include "smart45.h"
#include "smart135.h"
#include "smart90.h"
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

CYCLOGRAM(DELAY_025S){
    constexpr uint16_t DELAY_TIME = 250; // ms 

    ms->v_f0 = 0;
    ms->theta_i0 = 0;

    if(s->time > DELAY_TIME){
        ms->isComplete = 1;
    }
    else ms->isComplete = 0;
}

CYCLOGRAM(FWD_HALF)
{
    ms->v_f0 = FORWARD_SPEED * FWD_SPEED_MULTIPLIER;
    FWD_default(ms, s, ms->theta_0);

    if(s->odometry->getRelativeDist() > HALF(CELL_SIZE) )
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(FWD_X)
{
    constexpr float acceleration = FWD_ACCELERATION; 
    constexpr float v0 = FORWARD_SPEED * FWD_SPEED_MULTIPLIER;
    static Integrator v = v0;
    
    
    
    if(s->odometry->getRelativeDist() < HALF(CELL_SIZE * x))
    {
        v.tick(acceleration);
    }
    else if(v.getOut() > v0)
    {
        v.tick(-acceleration);
    }

    ms->v_f0 = min(v.getOut(), MAX_FWD_SPEED_AFTER_ACC);
    FWD_default(ms, s, ms->theta_0);

    if(s->odometry->getRelativeDist() > CELL_SIZE * x)
    {
        //s->odometry->setTheta(ms->theta_0);
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(FWDE)
{
    ms->v_f0 = FORWARD_SPEED * FWDE_SPEED_MULTIPLIER;
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

    const int16_t LEFT_TRASHHOLD = s->optocoupler->getLeftTreshold();
    const int16_t RIGHT_TRASHHOLD = s->optocoupler->getRightTreshold();

    const float regulatorArray[4] = {
        0,//ни один не видит стену
        ANGLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * (right_sense - RIGHT_TRASHHOLD - OPTOCOUPLER_SENSE_ERROR),//стену видит только правый
        ANGLE_SPEED_OPTOCOUPLER_ONESEN_REG_K * (LEFT_TRASHHOLD + OPTOCOUPLER_SENSE_ERROR - left_sense),//стену видит только левый
        ANGLE_SPEED_OPTOCOUPLER_TWOSEN_REG_K * (right_sense - left_sense),//оба датчика
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
            CAN_TRY_ALIGN = 0;
            NEED_ALIGN = 1;
            dist_buf = s->odometry->getRelativeDist();
    }

    prev_left_wall_state = toBool(cell_from_sensors.west_wall);
    prev_right_wall_state = toBool(cell_from_sensors.east_wall);

    if(NEED_ALIGN)
    {
        if(s->odometry->getRelativeDist() - dist_buf > FROM_ZERO_WALL_TO_SIDE){
            ms->isComplete = true;
        }
    }
    else if(s->odometry->getRelativeDist() > CELL_SIZE)
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

        //s->odometry->setTheta(ms->theta_0);
    }
}

CYCLOGRAM(DIAG_X)
{
    ms->v_f0 = FAST_FORWARD_SPEED * FWD_SPEED_MULTIPLIER;

    
    #if USE_ANGLE_REGULATOR
        ms->theta_i0 = getThetaIfromAngleReg(s, ms->theta_0);
    #else
        ms->theta_i0 = 0;
    #endif

    if(s->odometry->getRelativeDist() > CELL_SIZE / M_SQRT2 * x)
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
    
    if(s->odometry->getRelativeDist() < forwDist)
    {
        FWD_default(ms, s, ms->theta_0);
    }
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() < HALF_PI)
    #else
    else if(s->odometry->getRelativeDist() > forwDist && s->odometry->getRelativeDist() < forwDist + circleDist)
    #endif
    {
        ms->theta_i0 = theta_i;
    }
    else
    {
        FWD_default(ms, s, ms->theta_0 + HALF_PI);
    }
    if(s->odometry->getRelativeDist() > 2 * forwDist + circleDist)
    {
        ms->isComplete = true;
        ms->theta_i0 += HALF_PI;
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
    
    if(s->odometry->getRelativeDist() < forwDist)
    {
        FWD_default(ms, s, ms->theta_0);
    }
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() > -HALF_PI)
    #else
    else if(s->odometry->getRelativeDist() > forwDist && s->odometry->getRelativeDist() < forwDist + circleDist)
    #endif
    {
        ms->theta_i0 = -theta_i;
    }
    else
    {
        FWD_default(ms, s, ms->theta_0 - HALF_PI);
    }
    if(s->odometry->getRelativeDist() > 2 * forwDist + circleDist)
    {
        ms->isComplete = true;
        ms->theta_i0 -= HALF_PI;
    }
    else ms->isComplete = false;

}

CYCLOGRAM(SS90SL)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float R = SS90S_TURN_RADIUS; //радиус поворота
    constexpr float theta_i = FAST_FORWARD_SPEED / R;

    constexpr float forwDist = CELL_SIZE - R;
    constexpr float circleDist = (TWO_PI * R) / 4;

    if(s->odometry->getRelativeDist() < forwDist)
    {
        FWD_default(ms, s, ms->theta_0);
    }
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() < HALF_PI)
    #else
    else if(s->odometry->getRelativeDist() > forwDist && s->odometry->getRelativeDist() < forwDist + circleDist)
    #endif
    {
        ms->theta_i0 = theta_i;
    }
    else
    {
        FWD_default(ms, s, ms->theta_0 + HALF_PI);
    }

    if(s->odometry->getRelativeDist() > 2 * forwDist + circleDist)
    {
        ms->isComplete = true;
        ms->theta_i0 += HALF_PI;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(SS90SR)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float R = SS90S_TURN_RADIUS; //радиус поворота
    constexpr float theta_i = FAST_FORWARD_SPEED / R;

    constexpr float forwDist = CELL_SIZE - R;
    constexpr float circleDist = (TWO_PI * R) / 4;

    if(s->odometry->getRelativeDist() < forwDist)
    {
        FWD_default(ms, s, ms->theta_0);
    }
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() > -HALF_PI)
    #else
    else if(s->odometry->getRelativeDist() > forwDist && s->odometry->getRelativeDist() < forwDist + circleDist)
    #endif
    {
        ms->theta_i0 = -theta_i;
    }
    else
    {
        FWD_default(ms, s, ms->theta_0 - HALF_PI);
    }

    if(s->odometry->getRelativeDist() > 2 * forwDist + circleDist)
    {
        ms->isComplete = true;
        ms->theta_i0 -= HALF_PI;
    }
    else ms->isComplete = false;
}
CYCLOGRAM(DD90SL)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float R = DD90S_TURN_RADIUS; //радиус поворота
    constexpr float forwDist = CELL_SIZE / M_SQRT2 - R;
    constexpr float circleDist = (2 * PI * R) / 4; // 90 = четверть окружности
    float theta_i = FAST_FORWARD_SPEED / R;

    #if USE_ANGLE
    if(s->odometry->getRelativeTheta() < HALF_PI) ms->theta_i0 = theta_i;
    #else
    if(s->odometry->getRelativeDist() > forwDist && s->odometry->getRelativeDist() < forwDist + circleDist) ms->theta_i0 = theta_i;
    #endif
    else ms->theta_i0 = 0;

    if(s->odometry->getRelativeDist() > forwDist + circleDist + forwDist)
    {
        ms->isComplete = true;
        ms->theta_i0 += HALF_PI;
    }
    else ms->isComplete = false;
}
CYCLOGRAM(DD90SR)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float R = DD90S_TURN_RADIUS; //радиус поворота
    constexpr float forwDist = CELL_SIZE / M_SQRT2 - R;
    constexpr float circleDist = (2 * PI * R) / 4; // 90 = четверть окружности
    float theta_i = FAST_FORWARD_SPEED / R;

    #if USE_ANGLE
    if(s->odometry->getRelativeTheta() > -HALF_PI) ms->theta_i0 = -theta_i;
    #else
    if(s->odometry->getRelativeDist() > forwDist && s->odometry->getRelativeDist() < forwDist + circleDist) ms->theta_i0 = -theta_i;
    #endif
    else ms->theta_i0 = 0;

    if(s->odometry->getRelativeDist() > forwDist + circleDist + forwDist)
    {
        ms->isComplete = true;
        ms->theta_i0 -= HALF_PI;
    }
    else ms->isComplete = false;
}


CYCLOGRAM(SS180SL)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float R = CELL_SIZE / 2;
    constexpr float theta_i = FAST_FORWARD_SPEED / R;
    constexpr float circleDist = PI * R; // 180 = половина окружности
    constexpr float forwDist = SS180S_FORW_DIST;

    if(s->odometry->getRelativeDist() < forwDist)
    {
        FWD_default(ms, s, ms->theta_0);
    }
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() < PI)
    #else
    else if(s->odometry->getRelativeDist() > forwDist && s->odometry->getRelativeDist() < forwDist + circleDist)
    #endif
    {
        ms->theta_i0 = theta_i;
    }
    else
    {
        FWD_default(ms, s, ms->theta_0 + PI);
    }
    if(s->odometry->getRelativeDist() > 2 * forwDist + circleDist)
    {
        ms->isComplete = true;
        ms->theta_i0 += PI;
    }
    else ms->isComplete = false;
}
CYCLOGRAM(SS180SR)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float R = CELL_SIZE / 2;
    constexpr float theta_i = FAST_FORWARD_SPEED / R;
    constexpr float circleDist = PI * R; // 180 = половина окружности
    constexpr float forwDist = SS180S_FORW_DIST;

    if(s->odometry->getRelativeDist() < forwDist)
    {
        FWD_default(ms, s, ms->theta_0);
    }
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() > -PI)
    #else
    else if(s->odometry->getRelativeDist() > forwDist && s->odometry->getRelativeDist() < forwDist + circleDist)
    #endif
    {
        ms->theta_i0 = -theta_i;
    }
    else
    {
        FWD_default(ms, s, ms->theta_0 - PI);
    }
    if(s->odometry->getRelativeDist() > 2 * forwDist + circleDist)
    {
        ms->isComplete = true;
        ms->theta_i0 -= PI;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(IP180)
{
    constexpr float theta_i = FORWARD_SPEED / (ROBOT_WIDTH / 2);
    constexpr float THETA_ERROR = 10 * PI / 180;

    ms->v_f0 = 0;
    ms->theta_i0 = theta_i;

    if(s->odometry->getRelativeTheta() >= PI - THETA_ERROR)
    {
        ms->isComplete = true;
        ms->theta_i0 += PI;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(IP90L)
{
    // constexpr float alpha = HALF_PI;
    // constexpr float theta = FORWARD_SPEED / (ROBOT_WIDTH / 2);

    // constexpr float t_0 = alpha / theta;
    // constexpr float saturation = 0.1; // [0 > saturation < 1]
    // constexpr float k_a = HALF(saturation); 
    
    // constexpr float T = t_0 / (1 - k_a);
    // constexpr float a = k_a * T;

    // constexpr float angle_in_acc = theta * a;
    // constexpr float theta_error = 2 * PI / 180;

    // constexpr float acc = theta / a;

    // const float cur_theta = s->odometry->getRelativeTheta();

    // ms->v_f0 = 0;

    // float acc_i = 0;
    // if(cur_theta < angle_in_acc){
    //     acc_i = acc;
    // }
    // else if(cur_theta > (alpha - angle_in_acc)){
    //     acc_i = -acc;
    // }
    // else acc_i = 0;

    // ms->theta_i0 += acc_i;

    // Serial.print(cur_theta);
    // Serial.print('\t');
    // Serial.println(ms->theta_i0);

    // if(cur_theta >= alpha){
    //     ms->isComplete = 1;
    //     ms->theta_i0 = 0;
    // }
    // else ms->isComplete = 0;

    constexpr float theta_i = FORWARD_SPEED / (ROBOT_WIDTH / 2);
    constexpr float THETA_ERROR = 6 * PI / 180;

    ms->v_f0 = 0;
    ms->theta_i0 = theta_i;

    if(s->odometry->getRelativeTheta() >= HALF_PI - THETA_ERROR)
    {
        ms->theta_i0 = 0;
        ms->isComplete = true;
        ms->theta_i0 += HALF_PI;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(IP90R)
{
    constexpr float theta_i = -FORWARD_SPEED / (ROBOT_WIDTH / 2);
    constexpr float THETA_ERROR = 6 * PI / 180;

    ms->v_f0 = 0;
    ms->theta_i0 = theta_i;

    if(s->odometry->getRelativeTheta() <= -(HALF_PI - THETA_ERROR))
    {
        ms->theta_i0 = 0;
        ms->isComplete = true;
        ms->theta_i0 -= HALF_PI;
    }
    else ms->isComplete = false;
}

#endif // !_CYCLOGRAMS_H_