#include "Drivers/StatusSelector.h"

void StatusSelector::sense(){
    _adc_reading = analogRead(FUNCTION_PIN);
}

void StatusSelector::plan(){
    work_with_button();
    work_with_opto();
}   

void StatusSelector::work_with_button(){
    const bool _but_state = _adc_reading >= ADC_THRESHOLD;

    const bool _forward_but_front = _prev_but_state == 0 && _but_state == 1;
    const bool _is_press_but      = _prev_but_state == 1 && _but_state == 1;

    if(_forward_but_front){
        _timer = _cur_millis;
        nextStatus();
    } 

    if(_is_press_but){
        _time_in_press = _cur_millis - _timer;
        const bool _is_down_but = _time_in_press > NEED_TIME_TO_DOWN;

        if(_is_down_but) {
            _but_counter = static_cast<uint8_t>(ProgramStatus::NONE);
        }
    }
    else _timer = _cur_millis;

    _prev_but_state = _but_state;
}

void StatusSelector::work_with_opto(){
    const bool opto_state = toBool(_optocouplers->getRelativeCell().north_wall);

    const bool forward_opto_front = _prev_opto_state == 0 && opto_state == 1;
    const bool in_open_opto       = _prev_opto_state == 1 && opto_state == 1;
    const bool is_close_opto      = _prev_opto_state == 1 && opto_state == 0;

    _is_slide = 0;

    switch (_opto_previod)
    {
    case SLIDE:
        _opto_previod = ZERO;
        _is_slide = 1;
        break;

    case ZERO:
        if(forward_opto_front) _opto_previod = FRONT;
        break;

    case FRONT:
        if(in_open_opto) _opto_previod = IN_OPEN;
        else _opto_previod = ZERO;
        break;

    case IN_OPEN:
        if(in_open_opto);
        else if(is_close_opto) _opto_previod = SLIDE;
        else _opto_previod = ZERO;
        break;

    default:
        break;
    }

    _prev_opto_state = opto_state;
}

void StatusSelector::init(){
    pinMode(_INPUT_PIN, INPUT);
    _indicator->init();
}

void StatusSelector::tick(){
    sense();
    plan();

    _indicator->blink(static_cast<uint8_t>(ProgramStatus::SIZE) - _but_counter);
}

ProgramStatus StatusSelector::getStatus() const{   
    const ProgramStatus _status = static_cast<ProgramStatus>(_but_counter);
    return _status;
}

void StatusSelector::nextStatus(){
    _but_counter = (_but_counter + 1) % static_cast<uint8_t>(ProgramStatus::SIZE);
}

void StatusSelector::setNoneStatus(){
    _but_counter = 0;
}

bool StatusSelector::isSlideFromOpto(){
    return _is_slide;
}

void StatusSelector::passMillis(const uint32_t t){
    _cur_millis = t;
    _indicator->passMillis(t);
}