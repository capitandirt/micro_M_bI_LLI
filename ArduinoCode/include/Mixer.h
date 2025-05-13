#ifndef _MIXER_H_
#define _MIXER_H_

#include "Drivers/Servo.h"

struct MotionControlConnectionParams{
    Servo *leftServo;
    Servo *rightServo;
};

class Mixer : private MotionControlConnectionParams{
public:
    Mixer(MotionControlConnectionParams* mccp) : MotionControlConnectionParams(*mccp){} 

    void impactVelocity(float _theta, float _forward_velocity);

private:
    float _theta = 0;
    float _forward_velocity = 0; 

    float _rad_theta = 0;
    float _rad_forward_velocity = 0;

    float _w_left_motor;
    float _w_right_motor;

    void calc_forward_movement();
    void calc_angular_movement();
    
    void set_theta(float& _theta);
    void set_forward_velocity(float& _forward_velocity);
};

#endif // !_MIXER_H_