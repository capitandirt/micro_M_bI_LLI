#ifndef _OPTOCOUPLER_H_
#define _OPTOCOUPLER_H_

#include "Arduino.h"
#include "CellsTypes.h"

struct OprocouplerConnectionParams{
    const uint8_t EMITERS_FWD;
    const uint8_t EMITERS_SIDE;
    const uint8_t REC_RIGHT;
    const uint8_t REC_LEFT;
    const uint8_t REC_FWD_LEFT;
    const uint8_t REC_FWD_RIGHT;
    const uint8_t SENSE_THRESHOLD_FWD_L;
    const uint8_t SENSE_THRESHOLD_FWD_R;
    const uint8_t SENSE_THRESHOLD_RIGHT;
    const uint8_t SENSE_THRESHOLD_LEFT;
};

struct Sense_t{
    uint8_t left;
    uint8_t forward_l;
    uint8_t forward_r;
    uint8_t right;
};

struct Sense_mask_t{
    bool left;
    bool forward_l;
    bool forward_r;
    bool right;
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
    uint8_t& operator[](const From index){ return _sense[static_cast<uint8_t>(index)]; }

private:
    static constexpr uint8_t SENSORS_NUMBER = sizeof(Sense_t) / sizeof(uint8_t);
    union{
        Sense_t _from;
        uint8_t _sense[SENSORS_NUMBER];
    };
};

class OptocouplerSensors : public OprocouplerConnectionParams{ 
public:
    OptocouplerSensors(OprocouplerConnectionParams* ocp) : OprocouplerConnectionParams(*ocp){}

    void init();
    void tick();
    Sense_t getSense();
    Cell getCell(Direction robotDir);

    void printMask();
    void printSense();
private:
    void calc_sense_mask();

private:
    Sense_mask_t sense_mask;
    OptocouplerSense sense;
};

#endif // !_OPTOCOUPLER_H_