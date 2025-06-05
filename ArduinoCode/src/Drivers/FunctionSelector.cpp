#include "Drivers/FunctionalSelector.h"

void FunctionalSelector::tick()
{
    _adc_reading = analogRead(_PIN);
}

// numerating from 0
bool FunctionalSelector::isLever(uint8_t N){
    if(N > LEVER_SIZE){
        return false;
    }

    return (_selected >> N) & 0B1; 
}


uint16_t FunctionalSelector::getSelected(){
    return _selected;
}

void FunctionalSelector::decodeAdcReading(){
    const uint16_t ADC_VALUES_SIZE = 0b1 << LEVER_SIZE;
    const uint16_t adc_values[ADC_VALUES_SIZE] = {660, 647, 630, 614, 590, 570, 545, 522, 461,
                                429, 385, 343, 271, 212, 128, 44};
    
    if(_adc_reading > 1000)
    {
        _selected = ADC_VALUES_SIZE; 
        return;
    }
    
    for (uint16_t i = 0; i < ADC_VALUES_SIZE; i++) 
    {
        if (_adc_reading > (adc_values[i] + adc_values[i + 1]) / 2) 
        {
            _selected = i;
            return;
        }
    }
}
    