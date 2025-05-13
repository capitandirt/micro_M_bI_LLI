#ifndef _VELOCITY_ESTIMATOR_H_
#define _VELOCITY_ESTIMATOR_H_

#include "Config.h"
#include "Drivers/Encoder.h"

struct VelocityEstimatorConnectionParams
{
    Encoder *encoder;
};

class VelocityEstimator : private VelocityEstimatorConnectionParams{
public:
    VelocityEstimator(VelocityEstimatorConnectionParams *vecp) : VelocityEstimatorConnectionParams(*vecp){}

    void tick();
    float getW() const;

private:
    float _w = 0;
};


#endif // !_VELOCITY_ESTIMATOR_H_