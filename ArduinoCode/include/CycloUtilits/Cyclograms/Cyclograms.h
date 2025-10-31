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
    
    static bool FIRST_ENTRANCE = 1;

    static bool prev_left_wall = 0;
    static bool prev_right_wall = 0;
    
    const Cell cell_from_sensors = s->optocoupler->getRelativeCell();
    const float cur_dist = s->odometry->getRelativeDist();

    const bool left_wall = toBool(cell_from_sensors.west_wall);
    const bool right_wall = toBool(cell_from_sensors.east_wall);

    if(FIRST_ENTRANCE){
        prev_left_wall = left_wall;
        prev_right_wall = right_wall;

        FIRST_ENTRANCE = 0;
    }
    
    const uint8_t passed_cells = cur_dist / CELL_SIZE;

    const bool left_wall_forward_front = prev_left_wall == 0 && left_wall == 1;
    const bool right_wall_forward_front = prev_right_wall == 0 && right_wall == 1;

    if(left_wall_forward_front || right_wall_forward_front){
        const float upd_dist = passed_cells * CELL_SIZE + (HALF(CELL_SIZE) + CELL_SIZE - FROM_HI_WALL_TO_SIDE);
        s->odometry->setRelativeDist(upd_dist);
    }

    if(cur_dist < HALF(CELL_SIZE * x))
    {
        v.tick(acceleration);
    }
    else if(v.getOut() > v0)
    {
        v.tick(-acceleration);
    }

    ms->v_f0 = min(v.getOut(), MAX_FWD_SPEED_AFTER_ACC);
    FWD_default(ms, s, ms->theta_0);

    if(cur_dist > CELL_SIZE * x)
    {
        s->odometry->setTheta(ms->theta_0);
        FIRST_ENTRANCE = 1;
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(FWDE)
{
    ms->v_f0 = FORWARD_SPEED * FWDE_SPEED_MULTIPLIER;
    // ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0);

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

    FWD_default(ms, s, ms->theta_0);

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
        if(s->odometry->getRelativeDist() - dist_buf > FROM_HI_WALL_TO_SIDE){
            ms->isComplete = true;
        }
    }
    else if(s->odometry->getRelativeDist() > CELL_SIZE)
    {
        ms->isComplete = true;
    }
    else ms->isComplete = false;

    if(ms->isComplete){
        if(NEED_ALIGN){
            s->odometry->setTheta(ms->theta_0);
        }

        FIRST_ENTRANCE = 1;
        prev_left_wall_state = 0;
        prev_right_wall_state = 0;
        CAN_TRY_ALIGN = 0;
        NEED_ALIGN = 0;
    }
}

CYCLOGRAM(DIAG_X)
{
    ms->v_f0 = FAST_FORWARD_SPEED * FWD_SPEED_MULTIPLIER;

    #if USE_ANGLE_REGULATOR
        ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0);
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
    constexpr float theta_i = FORWARD_SPEED / R - 0.3;

    constexpr float forwDist = CELL_SIZE / 2 - R;
    constexpr float circleDist = (2 * PI * R) / 4;
    
    static enum SS90SL_S{
        FWD1,
        TURN,
        FWD2,
        FINISH
    } ss90sl_state = FWD1;

    if(ss90sl_state == FWD1){
        ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0);
        // FWD_default(ms, s, ms->theta_0);

        if(s->odometry->getRelativeDist() > forwDist - 0.01){
            ss90sl_state = TURN;
        }
    }

    if (ss90sl_state == TURN){
        #if USE_GYRO
        const float err = circle_mod(HALF_PI - s->odometry->getRelativeTheta());
        
        if(s->odometry->getRelativeTheta() > HALF_PI - 0.05){
            ss90sl_state = FWD2;
        }
        ms->theta_i0 = constrain(err * ANGLE_REG_EXPLORER_KP, -theta_i, theta_i);
        #else
        ms->theta_i0 = theta_i;
        if(s->odometry()->getRalativeDist() > forwDist + circleDist) ss90sl_state = FWD2;
        #endif
    }

    if(ss90sl_state == FWD2){
        ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0 + HALF_PI);
        // FWD_default(ms, s, ms->theta_0 + HALF_PI);

         if(s->odometry->getRelativeDist() > 2 * forwDist + circleDist - 0.01){
            ss90sl_state = FINISH;
        }
    }

    if(ss90sl_state == FINISH){
        ss90sl_state = FWD1;
        ms->isComplete = true;
        ms->theta_0 += HALF_PI;
    }
}

CYCLOGRAM(SS90ER)
{
    ms->v_f0 = FORWARD_SPEED;
    constexpr float R = SEARCH_TURN_RADIUS; //радиус поворота
    constexpr float theta_i = FORWARD_SPEED / R - 0.3;

    constexpr float forwDist = CELL_SIZE / 2 - R;
    constexpr float circleDist = (2 * PI * R) / 4;
    
    static enum SS90SL_S{
        FWD1,
        TURN,
        FWD2,
        FINISH
    } ss90sl_state = FWD1;

    if(ss90sl_state == FWD1){
        ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0);
        // FWD_default(ms, s, ms->theta_0);

        if(s->odometry->getRelativeDist() > forwDist - 0.01){
            ss90sl_state = TURN;
        }
    }

    if (ss90sl_state == TURN){
        #if USE_GYRO
        const float err = circle_mod(-HALF_PI - s->odometry->getRelativeTheta());
        
        ms->theta_i0 = constrain(err * ANGLE_REG_EXPLORER_KP, -theta_i, theta_i);
        if(s->odometry->getRelativeTheta() < -HALF_PI + 0.05){
            ss90sl_state = FWD2;
        }
        #else
        ms->theta_i0 = theta_i;
        if(s->odometry()->getRalativeDist() > forwDist + circleDist) ss90sl_state = FWD2;
        #endif
    }

    if(ss90sl_state == FWD2){
        ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0 - HALF_PI);
        // FWD_default(ms, s, ms->theta_0 - HALF_PI - 0.1);

         if(s->odometry->getRelativeDist() > 2 * forwDist + circleDist - 0.01){
            ss90sl_state = FINISH;
        }
    }

    if(ss90sl_state == FINISH){
        ss90sl_state = FWD1;
        ms->isComplete = true;
        ms->theta_0 -= HALF_PI;
    }
}

CYCLOGRAM(SS90SL)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float R = SS90S_TURN_RADIUS; //радиус поворота
    constexpr float theta_i = FAST_FORWARD_SPEED / R;

    constexpr float forwDist = CELL_SIZE - R;
    constexpr float circleDist = (TWO_PI * R) / 4;

    static enum SS90SL_S{
        FWD1,
        TURN,
        FWD2,
        FINISH
    } ss90sl_state = FWD1;

    if(ss90sl_state == FWD1){
        FWD_default(ms, s, ms->theta_0);

        if(s->odometry->getRelativeDist() > forwDist){
            ss90sl_state = TURN;
        }
    }

    if (ss90sl_state == TURN){
        ms->theta_i0 = theta_i;

        #if USE_GYRO
        if(s->odometry->getRelativeTheta() > HALF_PI){
            ss90sl_state = FWD2;
        }
        #else
        if(s->odometry->getRelativeDist() > forwDist + circleDist) ss90sl_state = FWD2;
        #endif
    }

    if(ss90sl_state == FWD2){
        FWD_default(ms, s, ms->theta_0 + HALF_PI);

         if(s->odometry->getRelativeDist() > 2*forwDist + circleDist){
            ss90sl_state = FINISH;
        }
    }

    if(ss90sl_state == FINISH){
        ms->isComplete = true;
        ms->theta_0 += HALF_PI;
    }
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
        ms->theta_0 -= HALF_PI;
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

    if(s->odometry->getRelativeDist() < forwDist) ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0);
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() < HALF_PI) ms->theta_i0 = theta_i;
    #else
    if(s->odometry->getRelativeDist() > forwDist && s->odometry->getRelativeDist() < forwDist + circleDist) ms->theta_i0 = theta_i;
    #endif
    else ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0 + HALF_PI);

    if(s->odometry->getRelativeDist() > forwDist + circleDist + forwDist)
    {
        ms->isComplete = true;
        ms->theta_0 += HALF_PI;
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

    if(s->odometry->getRelativeDist() < forwDist) ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0);
    #if USE_ANGLE
    else if(s->odometry->getRelativeTheta() > -HALF_PI) ms->theta_i0 = -theta_i;
    #else
    if(s->odometry->getRelativeDist() > forwDist && s->odometry->getRelativeDist() < forwDist + circleDist) ms->theta_i0 = -theta_i;
    #endif
    else ms->theta_i0 = getThetaIFromAngleReg(s, ms->theta_0 - HALF_PI);

    if(s->odometry->getRelativeDist() > forwDist + circleDist + forwDist)
    {
        ms->isComplete = true;
        ms->theta_0 -= HALF_PI;
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

    static enum SS180SR_S{
        FIRST_ENTRANCE = 0,
        FWD1,
        TURN90_1,
        TURN90_2,
        FWD2,
        FINISH
    } ss180sl_state = FWD1;

    static float start_angle = 0;

    if(ss180sl_state == FIRST_ENTRANCE){
        start_angle = s->odometry->getRelativeTheta();
        ss180sl_state = FWD1;
    }

    const float cur_angle = s->odometry->getRelativeTheta();
    const float cur_dist = s->odometry->getRelativeDist();

    switch (ss180sl_state)
    {
    case FWD1:
        FWD_default(ms, s, ms->theta_0);
        if(cur_dist >= forwDist) {
            ss180sl_state = TURN90_1;
        }   
        else break;
    case TURN90_1:
        ms->theta_i0 = theta_i;
        if(abs(cur_angle) >= HALF_PI){
            s->odometry->updateRelative();
            ss180sl_state = TURN90_2;
        }
        break;

    case TURN90_2:
        ms->theta_i0 = theta_i;
        if(abs(cur_angle) >= HALF_PI){
            s->odometry->updateRelative();
            ss180sl_state = FWD2;
        }
        break;

    case FWD2:
        FWD_default(ms, s, ms->theta_0 + PI);
        if(s->odometry->getRelativeDist() >= forwDist) ss180sl_state = FINISH;
        break;

    case FINISH:
        ms->isComplete = true;
        ms->theta_0 += PI;
        ss180sl_state = FWD1;
        break;

    default:
        break;
    }    
}

CYCLOGRAM(SS180SR)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    constexpr float R = CELL_SIZE / 2;
    constexpr float theta_i = FAST_FORWARD_SPEED / R;
    constexpr float circleDist = PI * R; // 180 = половина окружности
    constexpr float forwDist = SS180S_FORW_DIST;

    static enum SS180SR_S{
        FIRST_ENTRANCE = 0,
        FWD1,
        TURN90_1,
        TURN90_2,
        FWD2,
        FINISH
    } ss180sr_state = FWD1;

    static float start_angle = 0;

    if(ss180sr_state == FIRST_ENTRANCE){
        start_angle = s->odometry->getRelativeTheta();
        ss180sr_state = FWD1;
    }

    const float cur_angle = s->odometry->getRelativeTheta();
    const float cur_dist = s->odometry->getRelativeDist();

    switch (ss180sr_state)
    {
    case FWD1:
        FWD_default(ms, s, ms->theta_0);
        if(cur_dist >= forwDist) {
            ss180sr_state = TURN90_1;
        }   
        else break;
    case TURN90_1:
        ms->theta_i0 = -theta_i;
        if(abs(cur_angle) >= HALF_PI){
            s->odometry->updateRelative();
            ss180sr_state = TURN90_2;
        }
        break;

    case TURN90_2:
        ms->theta_i0 = -theta_i;
        if(abs(cur_angle) >= HALF_PI){
            s->odometry->updateRelative();
            ss180sr_state = FWD2;
        }
        break;

    case FWD2:
        FWD_default(ms, s, ms->theta_0 - PI);
        if(s->odometry->getRelativeDist() >= forwDist) ss180sr_state = FINISH;
        break;

    case FINISH:
        ms->isComplete = true;
        ms->theta_0 -= PI;
        ss180sr_state = FWD1;
        break;

    default:
        break;
    }    
}

CYCLOGRAM(IP180)
{
    ms->v_f0 = 0;
    constexpr float theta_end = PI;

    const float err = circle_mod(theta_end - s->odometry->getRelativeTheta());
    static Integrator I = 0;
    if(abs(I.getOut()) < (FORWARD_SPEED / (ROBOT_WIDTH / 2))) I.tick(err * ANGLE_REG_IN_PLACE_KI); 

    ms->theta_i0 = err * ANGLE_REG_IN_PLACE_KP;

    static uint16_t time0 = s->time;
    bool need_rotate = true;
    if(abs(abs(s->odometry->getRelativeTheta()) - abs(theta_end)) > 0.02)
    {
        time0 = s->time;
    } 
    if((s->time - time0) / 1000 > IP_CALIB_TIME) need_rotate = false; 
    // Serial.println("ip180l: " + String(ms->theta_i0) + " err: " + String(err));
    if(!need_rotate)
    {
        ms->isComplete = true;
        ms->theta_0 += PI;
        I = 0;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(IP90L)
{
    ms->v_f0 = 0;
    constexpr float theta_end = HALF_PI;

    const float err = circle_mod(theta_end - s->odometry->getRelativeTheta());
    static Integrator I = 0;
    if(abs(I.getOut()) < (FORWARD_SPEED / (ROBOT_WIDTH / 2) - 0.1)) I.tick(err * ANGLE_REG_IN_PLACE_KI); 

    ms->theta_i0 = err * ANGLE_REG_IN_PLACE_KP;

    static uint16_t time0 = s->time;
    bool need_rotate = true;
    if(abs(abs(s->odometry->getRelativeTheta()) - abs(theta_end)) > 0.02)
    {
        time0 = s->time;
    } 
    if((s->time - time0) / 1000 > IP_CALIB_TIME) need_rotate = false; 

    if(!need_rotate)
    {
        ms->isComplete = true;
        ms->theta_0 += HALF_PI;
        I = 0;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(IP90R)
{
    ms->v_f0 = 0;
    constexpr float theta_end = -HALF_PI;

    const float err = circle_mod(theta_end - s->odometry->getRelativeTheta());
    static Integrator I = 0;
    constexpr float theta_i = FORWARD_SPEED / (ROBOT_WIDTH / 2);
    if(abs(I.getOut()) < theta_i - 0.1) I.tick(err * ANGLE_REG_IN_PLACE_KI); 

    ms->theta_i0 = constrain(err * ANGLE_REG_IN_PLACE_KP + I.getOut(), -theta_i, theta_i);  

    static uint16_t time0 = s->time;
    bool need_rotate = true;
    if(abs(abs(s->odometry->getRelativeTheta()) - abs(theta_end)) > 0.02)
    {
        time0 = s->time;
    } 
    if((s->time - time0) / 1000 > IP_CALIB_TIME) need_rotate = false; 

    if(!need_rotate)
    {
        ms->isComplete = true;
        ms->theta_0 -= HALF_PI;
        I = 0;
    }
    else ms->isComplete = false;
}

#endif // !_CYCLOGRAMS_H_