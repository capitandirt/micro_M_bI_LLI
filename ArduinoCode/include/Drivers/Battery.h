#ifndef _BATTERY_H_
#define _BATTERY_H_

#include "Arduino.h"

class Battery{
public:
    Battery(const uint8_t PIN) : _PIN(PIN) {}
    float volts();
    void tick();

private:
    static constexpr float MAX_VOLTAGE = 9.2;
    static constexpr float MAX_ADC = 660;
    static constexpr float DIVIDER_RATION = MAX_VOLTAGE/MAX_ADC;

    const uint8_t _PIN;
    float _voltage = 0;
};


#endif // !_BATTERY_H_