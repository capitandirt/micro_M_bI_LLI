#include "PiReg.h"

void PiReg::passSet(float& set){
    _set = set;

}

void PiReg::passCur(float& cur){
    _cur = cur;
}

float PiReg::getU() const{
    return _u;
}

void PiReg::tick(){
    _err = _set - _cur;

    _P = _err * Kp;
    _I = _integrator * Ki;

    _u = _P + _I;

    const float MAX_U = battery->getVoltage();
    _constrained_u = constrain(_u, -MAX_U, MAX_U);

    if(_u == _constrained_u  || _err * _u < 0){
        _integrator += _err * Ts_s;
    }
    else{
        _u = _constrained_u;
    }
}

void PiReg::reload(){
    // _integrator = 0;
}