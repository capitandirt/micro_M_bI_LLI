#include "Drivers/Led.h"

void Led::init(){
    pinMode(_PIN, OUTPUT);
}

void Led::blink(const uint8_t n){
    const uint32_t T = TIME_FULL_SATURATION / n;

    if(_cur_millis - _last_millis > T){
        _last_millis = _cur_millis;
    }
    else if(_cur_millis - _last_millis < T / 2){
        digitalWrite(_PIN, 0);
    }
    else{
        digitalWrite(_PIN, 1);
    }
}

void Led::passMillis(const uint32_t t){
    _cur_millis = t;
}