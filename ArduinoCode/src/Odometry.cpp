#include "Odometry.h"

float Odometry::getX() const{
    return X.getOut();
}

float Odometry::getY() const{
    return Y.getOut();
}

float Odometry::getTheta() const{
    return Theta.getOut();
}

float Odometry::getDist() const{
    return Distance.getOut();
}

Vec2 Odometry::getMazeCoord() const{
    return mazeCoord;
}

Direction Odometry::getDir() const{
    return dir;
}

void Odometry::update(float omegaL, float omegaR)
{
    vL = omegaL * WHEEL_RADIUS;
    vR = omegaR * WHEEL_RADIUS;

    float theta_i = (vR - vL) / ROBOT_WIDTH; //в этом вычислении опускается тангенс угла, тк tg(x) ~= x на малых углах
    Theta.tick(theta_i);

    v = (vR + vL) / 2;
    vX = v * cos(Theta.getOut());
    vY = v * sin(Theta.getOut());
    //PRINTLN(v);
    Distance.tick(v);
    X.tick(vX);
    Y.tick(vY);
}

void Odometry::updateDir(Direction dir)
{
    this->dir = dir;
}

void Odometry::reset()
{
    vL = 0;
    vR = 0;
    vX = 0;
    vY = 0;
    v = 0;
    mazeCoord = {0, 0};
    dir = Direction::N;
    X.reset();
    Y.reset();
    Theta.reset();
    Distance.reset();
}