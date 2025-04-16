#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "Encoder.h"

// struct OdometryConnectionParams{
//     Encoder* l_encoder;
//     Encoder* r_encoder;
// };

// class Odometry : public OdometryConnectionParams{
// private:
//     float dS_r, dS_l, dS_f, dAngle;

//     float x, y, angle;
// public:

//     Odometry(OdometryConnectionParams *ocp) : OdometryConnectionParams(*ocp){}
//     void tick();
// };
class Integrator
{
private:
    float private_out = 0;
public:
    const float& out = private_out;
    void tick(float in)
    {
        private_out += in * Ts_s;
    }
    void reset()
    {
        private_out = 0;
    }
};

class Odometry
{
private:
    Integrator
    X,
    Y,
    Theta,
    Distance;

    float
    vL = 0,
    vR = 0,
    vX,
    vY,
    v;

public:
    
    float getX() {return X.out;}
    float getY() {return Y.out;}
    float getTheta() {return Theta.out;}
    float getDist() {return Distance.out;}

    void update(float omegaL, float omegaR);
    void reset();
};

#endif // !_ODOMETRY_H_