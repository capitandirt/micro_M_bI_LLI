#ifndef _VELOCITY_ESTIMATOR_H_
#define _VELOCITY_ESTIMATOR_H_

#include "Config.h"
#include "Encoder.h"

struct VelocityEstimatorConnectionParams
{
    Encoder *encoder;
};

class VelocityEstimator : public VelocityEstimatorConnectionParams{
public:
    VelocityEstimator(VelocityEstimatorConnectionParams *vecp) : VelocityEstimatorConnectionParams(*vecp){}

    void tick();
    float getW() const;

private:
    static constexpr float LPF_K = 0.45;
    float _w = 0;
};


#endif // !_VELOCITY_ESTIMATOR_H_