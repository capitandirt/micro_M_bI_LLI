#ifndef _PI_REGULATOR_H_
#define _PI_REGULATOR_H_

#include "Config.h"
#include "VelocityEstimator.h"
#include "Arduino.h"

struct PiRegConnectionParams{
    float Kp;
    float Ki;
};

class PiReg : public PiRegConnectionParams{
private:
    float integrator = 0;
    
    float set = 0;
    float cur = 0;

    float err = 0;
    float P = 0;
    float I = 0;

    float u = 0;
    float constrain_u = 0;
public:
    PiReg(PiRegConnectionParams *prcp) : PiRegConnectionParams(*prcp){}     

    void init();
    void passSet(float& _set);
    void passCur(float& _cur);  
    float getU() const;

    void tick(); 
};

#endif // !_PI_REGULATOR_H_