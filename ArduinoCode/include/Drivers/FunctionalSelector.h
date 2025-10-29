#ifndef _FUCNTIONAL_SELECTOR_H
#define _FUCNTIONAL_SELECTOR_H

#include <Arduino.h>

/*
--------------------- слайдер внизу - 0, вверху - 1
|   1   2   3   4   |
|                   |
|   1   1   1   1   |
|   0   0   0   0   |
---------------------

1 - режим робота (0 - исследует от старта к финишу, 1 - едет фасты)

2

3 - нужно ли возвращаться с финиша (0 - надо, 1 - не надо)

4 - направление старта (0 - восток, 1 - юг)

*/


class FunctionalSelector
{
public:
    FunctionalSelector(uint8_t PIN): _PIN(PIN){} 

    void tick();
    void decodeAdcReading();
    
    bool isLever(uint8_t N);
    uint16_t getSelected();
    
private:
    static constexpr uint8_t LEVER_SIZE = 4;

private:
    const uint8_t _PIN;
    uint8_t _selected = 0;
    uint16_t _adc_reading = 0;
};

#endif // !_FUCNTIONAL_SELECTOR_H