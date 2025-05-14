#ifndef _FUNCTIONAL_SELECTOR_H_
#define _FUNCTIONAL_SELECTOR_H_

#include "Arduino.h"
#include "Config.h"

// this is fucking trash. fuck function selector, fuck 
class FunctionalSelector
{
private:
    uint8_t _adc_reading_states[16] = {660 / 4, 647 / 4, 630 / 4, 614 / 4, 590 / 4, 570 / 4,
                                       545 / 4, 522 / 4, 461 / 4, 429 / 4, 385 / 4, 343 / 4,
                                       271 / 4, 212 / 4, 128 / 4, 44 / 4};
    uint16_t _adc_reading = 0;
    uint8_t _state = 0;

    const uint8_t _PIN;
    void _get_state() 
    {
        for (int i = 0; i < 16; i++) 
        {
            if (_adc_reading > (_adc_reading_states[i] + _adc_reading_states[i + 1]) / 2) 
            {
                _state = i;
                break;
            }
        }
    }
public:
    FunctionalSelector(const uint8_t PIN) : _PIN(PIN) {}

    void init(){
        pinMode(_PIN, OUTPUT);
    }

    void tick()
    {
        _adc_reading = analogRead(FUNCTION_PIN);
        // _get_state();

        Serial.println(_adc_reading);
    }
};

#endif // !_FUNCTIONAL_SELECTOR_H_