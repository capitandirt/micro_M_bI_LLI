#include "Drivers/Led.h"

void Led::init(){
    pinMode(_PIN, OUTPUT);
}

void Led::tick(){
    if(_blink_value > 0){
        const uint32_t T = TIME_FULL_SATURATION / _blink_value;

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
}

void Led::toggle(){
    static bool state = 0;
    state = !state;
    digitalWrite(_PIN, state);
}

void Led::on(){
    digitalWrite(_PIN, 1);
}

void Led::off(){
    digitalWrite(_PIN, 0);
}

void Led::blink(const uint8_t n){
    _blink_value = n;
    if(_blink_value == 0) digitalWrite(_PIN, 0);
}

void Led::passMillis(const uint32_t t){
    _cur_millis = t;
}