#include "Servo.h"

void Servo::SetW(float _w){
    w = _w;
    w_PiReg->passSet(_w);
}

void Servo::init(){
    w_PiReg->init();
}

void Servo::act(){
    motor->drive(w_PiReg->getU());
}

void Servo::tick(){
    w_PiReg->tick();

    cur_w = velocityEstimator->GetW();
    w_PiReg->passCur(cur_w);

    act();
}

