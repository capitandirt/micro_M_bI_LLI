#ifndef _SLIDE_CATCHER_H
#define _SLIDE_CATCHER_H

#include "OptocouplerSensors.h"

class SlideCatcher{
public:
    SlideCatcher(OptocouplerSensors* optocouplers) :
        _optocouplers(optocouplers){}

    bool isSlide();
    void tick();

private:
    OptocouplerSensors* const _optocouplers;

    enum OptoPeriod { ZERO, FRONT, IN_ON, CLOSE, SLIDE } 
        _opto_previod; 

    bool _prev_opto_state = 0;
    bool _is_slide = 0;
};

#endif // !_SLIDE_CATCHER_H