#include "Drivers/FunctionalSelector.h"

void FunctionalSelector::sense(){
    _adc_reading = analogRead(FUNCTION_PIN);
}

void FunctionalSelector::plan(){
    _state = _adc_reading >= ADC_THRESHOLD;

    _forward_front = _prev_state == 0 && _state == 1;
    _is_press      = _prev_state == 1 && _state == 1;

    if(_forward_front){
        _timer = _cur_millis;
        _counter = (_counter + 1) % static_cast<uint8_t>(SelectorStatus::SIZE);
    } 

    if(_is_press){
        _time_in_press = _cur_millis - _timer;
        _is_down = _time_in_press > NEED_TIME_TO_DOWN;

        if(_is_down) {
            _counter = static_cast<uint8_t>(SelectorStatus::NONE);
            _timer = _cur_millis;
        }
    }
    else _timer = _cur_millis;

    _prev_state = _state;
}

void FunctionalSelector::init(){
    pinMode(_PIN, INPUT);
}

void FunctionalSelector::tick(){
    sense();
    plan();
}

SelectorStatus FunctionalSelector::getStatus() const{   
    _status = static_cast<SelectorStatus>(_counter);
    return _status;
}

void FunctionalSelector::passMillis(const uint32_t t){
    _cur_millis = t;
}