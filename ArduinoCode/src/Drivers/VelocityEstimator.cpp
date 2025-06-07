#include "Drivers/VelocityEstimator.h"

void VelocityEstimator::tick(){ 
    const float cur_w = encoder->getDPhi() / Ts_s;
    const float old_w = _w;

    _w += (cur_w - old_w) * LPF_K;
}

float VelocityEstimator::getW() const{
    return _w;
}