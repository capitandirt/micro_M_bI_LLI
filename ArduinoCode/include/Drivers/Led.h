#ifndef _LED_H_
#define _LED_H_

#include "Arduino.h"

class Led{
public:
    static constexpr uint32_t TIME_FULL_SATURATION = 1500; // ms
    
    Led(const uint8_t PIN) : _PIN(PIN) {}  

    void init();
    void tick();

    void blink(const uint8_t n);

    void passMillis(const uint32_t t);

private:
    uint8_t _blink_value;

    uint32_t _cur_millis  = 0;
    uint32_t _last_millis = 0;

    const uint8_t _PIN;
};

#endif // !_LED_H_