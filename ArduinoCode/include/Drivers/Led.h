#ifndef _LED_H_
#define _LED_H_

#include "Arduino.h"

class Led{
public:
    Led(const uint8_t PIN) : _PIN(PIN) {}  

    void init();
    void blink(const uint8_t n);

    void passMillis(const uint32_t t);

private:
    static constexpr uint32_t TIME_FULL_SATURATION = 4000; // ms

    uint32_t _cur_millis  = 0;
    uint32_t _last_millis = 0;

    const uint8_t _PIN;
};

#endif // !_LED_H_