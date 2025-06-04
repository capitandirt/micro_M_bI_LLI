#include <Arduino.h>
#include "config.h"

/*

--------------------- слайдер внизу - 0, вверху - 1
|   1   2   3   4   |
|                   |
|   1   1   1   1   |
|   0   0   0   0   |
---------------------

1 - режим робота (0 - исследует от старта к финишу, 1 - едет фасты)

2

3

4 - направление старта (0 - восток, 1 - юг)

*/



class FunctionalCelector
{
private:
    uint8_t functionOut = 0;
    int decodeFunctionSwitch(int functionValue) 
    {
        const int adcReading[] = {660, 647, 630, 614, 590, 570, 545, 522, 461,
                                    429, 385, 343, 271, 212, 128, 44,  0};
        if(functionValue > 1000)
        {
            return 16; 
        }
        int result = 16;
        for (int i = 0; i < 16; i++) 
        {
            if (functionValue > (adcReading[i] + adcReading[i + 1]) / 2) 
            {
                result = i;
                break;
            }
        }
        return result;
    }
public:
    uint8_t& function = functionOut;
    void tick()
    {
        int functionValue = analogRead(FUNCTION_PIN);
        functionOut = decodeFunctionSwitch(functionValue);
        if (functionOut == 16) 
        {
            digitalWrite(LED_BUILTIN, 1);
        } 
        else
        {
            digitalWrite(LED_BUILTIN, 0);
        }
    }
};