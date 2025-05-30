#include "Battery.h"

const float Battery::volts() {return batteryVolts;}

Battery& Battery::tick() 
{
    const float batteryDividerRatio = 2.0f;
    int adcValue = analogRead(BATTERY_VOLTS);
    float volts = adcValue * (5.0f * batteryDividerRatio / 1023.0f);
    batteryVolts = volts;
    return *this;
}