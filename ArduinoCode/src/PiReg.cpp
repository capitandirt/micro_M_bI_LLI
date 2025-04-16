#include "PiReg.h"

void PiReg::init(){
    return;
}

void PiReg::passSet(float& _set){
    set = _set;
    if(set == 0){
        integrator = 0;
    }
    
}

void PiReg::passCur(float& _cur){
    cur = _cur;
}

float PiReg::getU() const{
    return u;
}

void PiReg::tick(){
    err = set - cur;

    P = err * Kp;
    I = integrator * Ki;

    u = P + I;

    constrain_u = constrain(u, NEG_MAX_U, MAX_U);
    if(u == constrain_u){
        integrator += err * Ts_s;
    }
    else{
        u = constrain_u;
    }
}
