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

    _P = _err * KP;
    _I = _integrator * KI;

    _u = _P + _I;

    _constrained_u = constrain(_u, NEG_MAX_U, MAX_U);

    if(_u == _constrained_u){
        _integrator += _err * TS_S;
    }
    else{
        _u = _constrained_u;
    }
}

void PiReg::reload(){
    // _integrator = 0;
}