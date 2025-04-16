#ifndef _VELOCITY_ESTIMATOR_H_
#define _VELOCITY_ESTIMATOR_H_

#include "Config.h"
#include "Encoder.h"

struct VelocityEstimatorConnectionParams
{
    Encoder *encoder;
};

class VelocityEstimator : public VelocityEstimatorConnectionParams{
private:
    float w = 0;

    float phi; 
    float phi_old = 0;
public:
    VelocityEstimator(VelocityEstimatorConnectionParams *vecp) : VelocityEstimatorConnectionParams(*vecp){}

    void init();    
    void tick();

    float GetW();
};


#endif // !_VELOCITY_ESTIMATOR_H_