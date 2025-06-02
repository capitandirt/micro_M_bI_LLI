#ifndef _SERVO_H_
#define _SERVO_H_

#include "Encoder.h"
#include "Motor.h"
#include "PiReg.h"

struct ServoConnectionParams{
    PiReg* const w_PiReg;
    Motor* const motor;
    VelocityEstimator* const velocityEstimator;
};

class Servo : private ServoConnectionParams{
private:
    float _w;
    float cur_w;

    void act();
public:
    Servo(ServoConnectionParams *scp) : ServoConnectionParams(*scp){}
    
    void tick();

    void setW(const float _w);
};

#endif // !_SERVO_H_ 
