#include "Mixer.h"

void Mixer::calc_forward_movement(){
    _rad_forward_velocity = _forward_velocity / WHEEL_RADIUS;
}

void Mixer::calc_angular_movement(){
    _rad_theta = (ROBOT_WIDTH / WHEEL_RADIUS) * _theta;
}

void Mixer::set_theta(float& theta){
    _theta = theta;

    calc_angular_movement();
}

void Mixer::set_forward_velocity(float& forward_velocity){
    _forward_velocity = forward_velocity;

    calc_forward_movement();
}

void Mixer::impactVelocity(float theta, float forward_velocity)
{   
    _theta = theta;
    _forward_velocity = forward_velocity;

    calc_angular_movement();
    calc_forward_movement();

    _w_left_motor = _rad_forward_velocity - HALF(_rad_theta);
    _w_right_motor = _rad_forward_velocity + HALF(_rad_theta);

    leftServo->setW(_w_left_motor);
    rightServo->setW(_w_right_motor);
}