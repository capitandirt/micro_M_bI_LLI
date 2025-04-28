#ifndef _OPTOCOUPLER_H_
#define _OPTOCOUPLER_H_

#include "Arduino.h"
#include "CellsTypes.h"

struct OprocouplerConnectionParams{
    uint8_t const EMITERS_A;
    uint8_t const EMITERS_B;
    const uint8_t SENSOR_0;
    const uint8_t SENSOR_1;
    const uint8_t SENSOR_2;
    const uint8_t SENSOR_3;
    const uint8_t SENSOR_4;
    const uint8_t SENSOR_5;
};

struct Sense_t{
    float left;
    float forward_l;
    float forward_r;
    float right;
};

struct OptocouplerSense{
public:
    enum class From : uint8_t
    {
        LEFT = 0,
        FORWARD_L,
        FORWARD_R,
        RIGHT,
    };
    static constexpr uint8_t getSenseSize(){ return SENSORS_NUMBER; }
    
    Sense_t get() const {return _from; }

    float operator[](const From index){ return _sense[static_cast<uint8_t>(index)]; }
private:
    static constexpr uint8_t SENSORS_NUMBER = sizeof(Sense_t) / sizeof(float);
    union{
        Sense_t _from;
        float _sense[SENSORS_NUMBER];
    };
};

class OptocouplerSensors : public OprocouplerConnectionParams{ 
public:
    OptocouplerSensors(OprocouplerConnectionParams* ocp) : OprocouplerConnectionParams(*ocp){}

    void init();
    void update();
    Sense_t getSense();
    Cell getCellFromSensors(Direction robotDir);

private:
    OptocouplerSense sense;
};

#endif // !_OPTOCOUPLER_H_