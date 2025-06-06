#include "Drivers/Battery.h"

float Battery::getVoltage() {
    return _voltage;
}

void Battery::tick() 
{
    const uint16_t adc_value = analogRead(_PIN);
    const float volts = adc_value * DIVIDER_RATION;
    _voltage += (volts - _voltage) * LPF_K;
}