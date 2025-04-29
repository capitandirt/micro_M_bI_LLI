#include "VelocityEstimator.h"

void VelocityEstimator::tick(){ 
    _w = encoder->getDPhi() / Ts_s;
}

float VelocityEstimator::getW() const{
    return _w;
}