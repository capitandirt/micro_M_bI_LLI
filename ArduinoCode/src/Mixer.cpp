#include "Mixer.h"

void Mixer::calc_forward_movement(){
    rad_forward_velocity = forward_velocity / WHEEL_RADIUS;
}

void Mixer::calc_angular_movement(){
    rad_theta = (ROBOT_WIDTH / WHEEL_RADIUS) * theta;
}

void Mixer::set_theta(float& _theta){
    theta = _theta;

    calc_angular_movement();
}

void Mixer::set_forward_velocity(float& _forward_velocity){
    forward_velocity = _forward_velocity;

    calc_forward_movement();
}

void Mixer::setMouseVelocity(float _theta, float _forward_velocity)
{   
    theta = _theta;
    forward_velocity = _forward_velocity;

    calc_angular_movement();
    calc_forward_movement();

    w_left_motor = rad_forward_velocity - HALF(rad_theta);
    w_right_motor = rad_forward_velocity + HALF(rad_theta);
    
    leftServo->SetW(w_left_motor);
    rightServo->SetW(w_right_motor);
}