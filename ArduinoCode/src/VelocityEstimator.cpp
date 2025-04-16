#include "VelocityEstimator.h"

void VelocityEstimator::init(){
    return;
}

void VelocityEstimator::tick(){ 
    w = encoder->GetDPhi() / Ts_s;
}

float VelocityEstimator::GetW(){
    return w;
}