#include "Odometry.h"


void Odometry::update(float omegaL, float omegaR)
{
    vL = omegaL * WHEEL_RADIUS;
    vR = omegaR * WHEEL_RADIUS;

    float theta_i = (vR - vL) / ROBOT_WIDTH; //в этом вычислении опускается тангенс угла, тк tg(x) ~= x на малых углах
    Theta.tick(theta_i);

    v = (vR + vL) / 2;
    vX = v * cos(Theta.out);
    vY = v * sin(Theta.out);

    Distance.tick(v);
    X.tick(vX);
    Y.tick(vY);
}

void Odometry::reset()
{
    vL = 0;
    vR = 0;
    vX = 0;
    vY = 0;
    v = 0;
    X.reset();
    Y.reset();
    Theta.reset();
    Distance.reset();
}