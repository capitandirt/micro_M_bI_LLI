#ifndef _MIXER_H_
#define _MIXER_H_

#include "Servo.h"

struct MotionControlConnectionParams{
    Servo *leftServo;
    Servo *rightServo;
};

class Mixer : public MotionControlConnectionParams{
private:
    float theta = 0;
    float forward_velocity = 0; 

    float rad_theta = 0;
    float rad_forward_velocity = 0;

    float w_left_motor;
    float w_right_motor;

    void calc_forward_movement();
    void calc_angular_movement();
    
    void set_theta(float& _theta);
    void set_forward_velocity(float& _forward_velocity);
public:
    Mixer(MotionControlConnectionParams* mccp) : MotionControlConnectionParams(*mccp){} 

    void setMouseVelocity(float _theta, float _forward_velocity);
};

#endif // !_MIXER_H_