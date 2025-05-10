#ifndef _ALIGNMTNT_H_
#define _ALIGNMTNT_H_

#include "../CycloTypes.h"
#include "../../OptocouplerSensors.h"
#include "Cyclogram.config.h"

CYCLOGRAM(TO_ALIGN)
{
    ms->v_f0 = -FORWARD_SPEED;
    if(s->time > ALIGNMENT_TIME)
    {
        ms->v_f0 = 0;
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

CYCLOGRAM(FROM_ALIGN_TO_CENTER)
{
    ms->v_f0 = FORWARD_SPEED;
    if(s->robotState->getDist() > FROM_ALIGNMENT_TO_CENTER)
    {
        ms->v_f0 = 0;
        ms->isComplete = true;
    }
    else ms->isComplete = false;
}

#endif // !_ALIGNMTNT_H_