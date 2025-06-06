#ifndef _BATTERY_H_
#define _BATTERY_H_

#include "Arduino.h"

class Battery{
public:
    Battery(const uint8_t PIN) : _PIN(PIN) {}
    float getVoltage();
    void tick();

private:
    static constexpr float PIN_VOLTAGE = 4.07;
    static constexpr float BATTERY_VOLTAGE = 9.18;
    static constexpr float MAX_VOLTAGE = 5;
    static constexpr float MAX_ADC = 1023;
    static constexpr float DIVIDER_RATION = BATTERY_VOLTAGE / PIN_VOLTAGE * MAX_VOLTAGE/MAX_ADC;
    static constexpr float LPF_K = 0.95;

    const uint8_t _PIN;
    float _voltage = 0;
};


#endif // !_BATTERY_H_