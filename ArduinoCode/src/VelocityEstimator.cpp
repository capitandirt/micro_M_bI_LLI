#include "VelocityEstimator.h"

void VelocityEstimator::tick(){ 
    _w = encoder->GetDPhi() / Ts_s;
}

float VelocityEstimator::getW() const{
    return _w;
}