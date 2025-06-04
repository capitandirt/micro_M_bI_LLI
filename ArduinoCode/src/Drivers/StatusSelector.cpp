#include "Drivers/StatusSelector.h"

void StatusSelector::sense(){
    _adc_reading = analogRead(FUNCTION_PIN);
}

void StatusSelector::plan(const uint8_t FUNCTION_SELECTOR_DATA){
    const bool _but_state = _adc_reading >= ADC_THRESHOLD;
    const bool _forward_but_front = _prev_but_state == 0 && _but_state == 1;

    if(_slideCatcher->isSlide() && (_counter == static_cast<uint8_t>(ProgramStatus::NEED_START_COMMAND))){
        if(FUNCTION_SELECTOR_DATA >= 8) setStatus(ProgramStatus::DELAY_BEFORE_FAST);
        else setStatus(ProgramStatus::DELAY_BEFORE_GO_FINISH);
    }

    if(_forward_but_front){
        if(_counter == static_cast<uint8_t>(ProgramStatus::FAST)) setStatus(ProgramStatus::NONE);
        else nextStatus();
    } 
    
    _prev_but_state = _but_state;
}   

void StatusSelector::init(){
    pinMode(_INPUT_PIN, INPUT);
}

void StatusSelector::tick(const uint8_t FUNCTION_SELECTOR_DATA){
    sense();
    plan(FUNCTION_SELECTOR_DATA);

    if(_cur_millis - _timer < TIME_IN_BLINK) _indicator->blink(4);
    else _indicator->blink(0);
}

ProgramStatus StatusSelector::getStatus() const{   
    const ProgramStatus _status = static_cast<ProgramStatus>(_counter);
    return _status;
}

void StatusSelector::setStatus(ProgramStatus s){
    _counter = static_cast<uint8_t>(s);
}

void StatusSelector::nextStatus(){
    _counter = (_counter + 1) % static_cast<uint8_t>(ProgramStatus::SIZE);
    _timer = _cur_millis;
}

void StatusSelector::setNoneStatus(){
    _counter = 0;
}

void StatusSelector::passMillis(const uint32_t t){
    _cur_millis = t;
}