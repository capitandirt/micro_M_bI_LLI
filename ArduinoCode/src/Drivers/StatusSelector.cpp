#include "Drivers/StatusSelector.h"

void StatusSelector::sense(){
    _adc_reading = analogRead(FUNCTION_PIN);
}

void StatusSelector::plan(){
    const bool _but_state = _adc_reading >= ADC_THRESHOLD;
    const bool _forward_but_front = _prev_but_state == 0 && _but_state == 1;

    if(_slideCatcher->isSlide() && _counter == static_cast<uint8_t>(ProgramStatus::NEED_START_COMMAND)){
        nextStatus();
    }

    if(_forward_but_front){
        if(_counter == static_cast<uint8_t>(ProgramStatus::NONE)) nextStatus();
        else _counter = static_cast<uint8_t>(ProgramStatus::NONE);
        
        _timer = _cur_millis;
    } 
    _prev_but_state = _but_state;
}   

void StatusSelector::init(){
    pinMode(_INPUT_PIN, INPUT);
}

void StatusSelector::tick(){
    sense();
    plan();

    _indicator->blink(static_cast<uint8_t>(ProgramStatus::SIZE) - _counter);
}

ProgramStatus StatusSelector::getStatus() const{   
    const ProgramStatus _status = static_cast<ProgramStatus>(_counter);
    return _status;
}

void StatusSelector::nextStatus(){
    _counter = (_counter + 1) % static_cast<uint8_t>(ProgramStatus::SIZE);
}

void StatusSelector::setNoneStatus(){
    _counter = 0;
}

void StatusSelector::passMillis(const uint32_t t){
    _cur_millis = t;
}