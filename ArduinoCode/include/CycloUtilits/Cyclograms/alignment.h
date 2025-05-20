#ifndef _ALIGNMTNT_H_
#define _ALIGNMTNT_H_

#include "CycloUtilits/CycloTypes.h"
#include "Cyclogram.config.h"

CYCLOGRAM(TO_BACK_ALIGN)
{
    ms->v_f0 = -FORWARD_SPEED;
    ms->theta_i0 = 0;

    if(s->time > BACK_ALIGNMENT_TIME)
    {
        ms->v_f0 = 0;
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(FROM_BACK_ALIGN_TO_CENTER)
{
    ms->v_f0 = FORWARD_SPEED;
    ms->theta_i0 = 0;

    if(s->odometry->getDist() > FROM_BACK_ALIGNMENT_TO_CENTER)
    {
        ms->v_f0 = 0;
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(TO_FORWARD_ALIGN)
{
    ms->v_f0 = FORWARD_SPEED;
    ms->theta_i0 = 0;

    if(s->time > FORWARD_ALIGNMENT_TIME){
        ms->v_f0 = 0;
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(FROM_FORWARD_ALIGN_TO_CENTER){
    ms->v_f0 = -FORWARD_SPEED;
    ms->theta_i0 = 0;

    if(s->odometry->getDist() < -FROM_FORWARD_ALIGNMENT_TO_CENTER)
    {
        ms->v_f0 = 0;
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

#endif // !_ALIGNMTNT_H_