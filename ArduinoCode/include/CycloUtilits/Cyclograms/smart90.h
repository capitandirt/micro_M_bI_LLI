#ifndef _SMART90_H_
#define _SMART90_H_

#include "CycloUtilits/CycloTypes.h"
#include "Cyclogram.config.h"


CYCLOGRAM(SS90AL)
{
    ms->v_f0 = FAST_FORWARD_SPEED;
    float jump = 0;
    static Integrator a = 0;
    static Integrator v = 0;

    float forwDist,
          transitionDist,
          circleDist;
          
    if(s->odometry->getRelativeDist() < forwDist) jump = 0;
    else if(s->odometry->getRelativeDist() > forwDist && s->odometry->getRelativeDist() < forwDist + transitionDist) jump = 0;
    else if(s->odometry->getRelativeDist() > forwDist + transitionDist && s->odometry->getRelativeDist() < forwDist + transitionDist + circleDist) jump = 0;

    a.tick(jump);
    v.tick(a.getOut());


}


#endif // !_SMART90_H_