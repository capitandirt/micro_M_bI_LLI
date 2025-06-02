#ifndef _PI_REGULATOR_H_
#define _PI_REGULATOR_H_

#include "Drivers/VelocityEstimator.h"
#include "Arduino.h"

struct PiRegConnectionParams{
    const float KP;
    const float KI;
    const float TS_S;
};

class PiReg : private PiRegConnectionParams{
public:
    PiReg(PiRegConnectionParams *prcp) : PiRegConnectionParams(*prcp){}     

    void passSet(float& set);
    void passCur(float& cur);  
    float getU() const;

    void reload();
    void tick(); 

private:
    float _integrator = 0;
    
    float _set = 0;
    float _cur = 0;

    float _err = 0;
    float _P = 0;
    float _I = 0;

    float _u = 0;
    float _constrained_u = 0;
};

#endif // !_PI_REGULATOR_H_