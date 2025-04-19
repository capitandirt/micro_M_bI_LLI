#include "Servo.h"

void Servo::setW(const float w){
    _w = w;
    w_PiReg->passSet(_w);
}

void Servo::act(){
    motor->drive(w_PiReg->getU());
}

void Servo::tick(){
    w_PiReg->tick();

    cur_w = velocityEstimator->getW();
    w_PiReg->passCur(cur_w);

    act();
}

