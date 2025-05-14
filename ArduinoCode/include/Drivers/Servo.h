#ifndef _SERVO_H_
#define _SERVO_H_

#include "Encoder.h"
#include "Motor.h"
#include "PiReg.h"

struct ServoConnectionParams{
    PiReg* w_PiReg;
    Motor* motor;
    VelocityEstimator* velocityEstimator;
};

class Servo : public ServoConnectionParams{
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
