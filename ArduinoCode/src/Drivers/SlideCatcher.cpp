#include "Drivers/SlideCatcher.h"

bool SlideCatcher::isSlide(){
    return _is_slide;
}

void SlideCatcher::tick(){
    const bool opto_state = toBool(_optocouplers->getRelativeCell().north_wall);

    const bool forward_opto_front = _prev_opto_state == 0 && opto_state == 1;
    const bool in_on_opto         = _prev_opto_state == 1 && opto_state == 1;
    const bool back_opto_front    = _prev_opto_state == 1 && opto_state == 0;

    _is_slide = 0;
    
    switch (_opto_previod)
    {
    case ZERO:
        if(forward_opto_front) _opto_previod = FRONT;
        break;

    case FRONT:
        if(in_on_opto) _opto_previod = IN_ON;
        else _opto_previod = ZERO;
        break;

    case IN_ON:
        if(in_on_opto);
        else if(back_opto_front) _opto_previod = SLIDE;
        else _opto_previod = ZERO;
        break;

    case SLIDE:
        _opto_previod = ZERO;
        _is_slide = 1;
        break;
    }

    _prev_opto_state = opto_state;
}