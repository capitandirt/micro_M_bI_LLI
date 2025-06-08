#ifndef _SMART90_H_
#define _SMART90_H_

#include "CycloUtilits/CycloTypes.h"
#include "Cyclogram.config.h"


CYCLOGRAM(SS90AL)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    static Integrator omega = 0;
    static Integrator X = 0;


    constexpr float a = 1.7;
    constexpr float T = 2;
    constexpr float theta_end = HALF_PI;
    const float t0 = (a * (float)T - sqrt(a*a*(float)(T*T) - 4*theta_end*a)) / (2 * a);
          
    if(s->time / 1000 < t0) omega.tick(a);
    else if(s->time / 1000 < T - t0);
    else if(s->time / 1000 < T) omega.tick(-a);

    X.tick(ms->v_f0 * cos(s->odometry->getTheta()));

    Serial.println("t: " + String(s->time) + " omega: " + String(omega.getOut()) + " theta: " + String(s->odometry->getTheta()));
    ms->theta_i0 = omega.getOut();
    if(s->time / 1000 > T)
    {
        ms->isComplete = true;
        Serial.println(X.getOut());
        ms->theta_i0 += HALF_PI;
    }
    else ms->isComplete = false;
}


#endif // !_SMART90_H_