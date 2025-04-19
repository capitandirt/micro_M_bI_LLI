#ifndef _OPTOCOUPLER_H_
#define _OPTOCOUPLER_H_

#include "Arduino.h"
#include "CellsTypes.h"

struct OprocouplecConnectionParams{
    
};

enum class OptocouplerSensorsState : uint8_t
{
    LEFT = 0,
    FORWARD_L,
    FORWARD_R,
    RIGHT,

    SENSORS_NUMBER
};

class OptocouplerSensors : public OprocouplecConnectionParams{ 
public:
    void update();
    void getDist(float* dist); //дист - массив из 4 значений на каждый датчик
    Cell getCellFromSensors(Direction robotDir);

private:
    float _dist[static_cast<uint8_t>(OptocouplerSensorsState::SENSORS_NUMBER)];
};

#endif // !_OPTOCOUPLER_H_