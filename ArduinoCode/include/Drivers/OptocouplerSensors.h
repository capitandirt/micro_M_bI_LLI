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
    int16_t SENSE_THRESHOLD_FWD_L;
    int16_t SENSE_THRESHOLD_FWD_R;
    int16_t SENSE_THRESHOLD_RIGHT;
    int16_t SENSE_THRESHOLD_LEFT;
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

class OptocouplerSensors : private OprocouplerConnectionParams{ 
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
    
    void setStaticError()            noexcept; 
    int16_t getStaticError()           const;

    int16_t getRightSense0()           const;
    int16_t getLeftSense0()            const;

    int16_t getLeftTreshold()          const;
    int16_t getRightTreshold()         const;

    
    private:
    void    calc_sense_mask();
    void    calc_relative_cell();
    
    private:
    static constexpr float K_F = 0.99;
    
    Cell _relative_cell;
    Sense_t dark_sense;

    volatile bool CAN_GET_SENSE = 0;

    Sense_mask_t _sense_mask;
    
    OptocouplerSense _sense;   
    
    int16_t _static_err = 0;
    int16_t _right_sense0 = 0; //значение датчика в середине клетки
    int16_t _left_sense0 = 0; //значение датчика в середине клетки
};

#endif // !_OPTOCOUPLER_H_