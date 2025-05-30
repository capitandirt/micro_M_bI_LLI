#ifndef _OPTOCOUPLER_H_
#define _OPTOCOUPLER_H_

#include "Arduino.h"
#include "CellsTypes.h"

struct OprocouplerConnectionParams{
    const uint16_t EMITERS_FWD;
    const uint16_t EMITERS_SIDE;
    const uint16_t REC_RIGHT;
    const uint16_t REC_LEFT;
    const uint16_t REC_FWD_LEFT;
    const uint16_t REC_FWD_RIGHT;
    const uint16_t SENSE_THRESHOLD_FWD_L;
    const uint16_t SENSE_THRESHOLD_FWD_R;
    const uint16_t SENSE_THRESHOLD_RIGHT;
    const uint16_t SENSE_THRESHOLD_LEFT;
};

struct Sense_t{
    int16_t left;
    int16_t forward_l;
    int16_t forward_r;
    int16_t right;
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
    
    Sense_t get() const { return _from; }
    uint16_t& operator[](const From index){ return _sense[static_cast<uint8_t>(index)]; }

private:
    static constexpr uint8_t SENSORS_NUMBER = sizeof(Sense_t) / sizeof(int16_t);
    union{
        Sense_t _from = {};
        uint16_t _sense[SENSORS_NUMBER];
    };
};

class OptocouplerSensors : public OprocouplerConnectionParams{ 
public:
    OptocouplerSensors(OprocouplerConnectionParams* ocp) : OprocouplerConnectionParams(*ocp){}

    void    init();
    void    tick();
    void    calc();

    Sense_t getSense()                  const;
    Cell    getRelativeCell()           const;
    Cell    getCell(Direction robotDir) const;

    void    printAbsCell()              const;
    void    printMask()                 const;
    void    printSense()                const;
    Sense_t dark_sense;

private:
    void    calc_sense_mask();
    void    calc_relative_cell();

private:
    static constexpr float K_F = 0.99;

    Cell _relative_cell;

    volatile bool CAN_GET_SENSE = 0;

    Sense_mask_t _sense_mask;
    
    OptocouplerSense _sense;    
};

#endif // !_OPTOCOUPLER_H_