#include "Drivers/ProgramStatusHandler.h"

void ProgramStatusHandler::sense(){
    _adc_reading = analogRead(FUNCTION_PIN);
}

void ProgramStatusHandler::plan(){
    const bool _but_state = _adc_reading >= ADC_THRESHOLD;
    const bool _back_but_front = _prev_but_state == 1 && _but_state == 0;

    if(_slideCatcher->isSlide() && (_counter == static_cast<uint8_t>(ProgramStatus::NEED_EXPLORER_COMMAND))){
        setStatus(ProgramStatus::DELAY_BEFORE_GO_FINISH);
    }

    if(_back_but_front){
        nextStatus();
    } 
    
    _prev_but_state = _but_state;
}   

void ProgramStatusHandler::init(){
    pinMode(_INPUT_PIN, INPUT);
}

void ProgramStatusHandler::tick(){
    sense();
    plan();

    if(_cur_millis - _timer < TIME_IN_BLINK) _indicator->blink(4);
    else _indicator->blink(0);
}

ProgramStatus ProgramStatusHandler::getStatus() const{   
    const ProgramStatus _status = static_cast<ProgramStatus>(_counter);
    return _status;
}

void ProgramStatusHandler::setStatus(ProgramStatus s){
    _counter = static_cast<uint8_t>(s);
}

void ProgramStatusHandler::nextStatus(){
    _counter = (_counter + 1) % static_cast<uint8_t>(ProgramStatus::SIZE);
    _timer = _cur_millis;
}

void ProgramStatusHandler::setNoneStatus(){
    _counter = 0;
}

void ProgramStatusHandler::passMillis(const uint32_t t){
    _cur_millis = t;
}