#include "Drivers/VelocityEstimator.h"

void VelocityEstimator::tick(){ 
    _w = _encoder->getDPhi() / Ts_s;
}

float VelocityEstimator::getW() const{
    return _w;
}