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

float Odometry::getRelativeX() const{
    return X - X_r;
}

float Odometry::getRelativeY() const{
    return Y - Y_r;
}

float Odometry::getRelativeTheta() const{
    return Theta - Theta_r;
}

float Odometry::getRelativeDist() const{
    return Distance - Distance_r;
}

Vec2 Odometry::getMazeCoords() const{
    return mazeCoord;
}

Direction Odometry::getDir() const{
    return dir;
}

void Odometry::printDir() const{
    switch (dir)
    {
    case Direction::N:
        Serial.println("N");
        break;
    case Direction::S:
        Serial.println("S");
        break;
    case Direction::E:
        Serial.println("E");
        break;
    case Direction::W:
        Serial.println("W");
        break;
    }
}

void Odometry::tick(float omegaL, float omegaR)
{
    vL = omegaL * WHEEL_RADIUS;
    vR = omegaR * WHEEL_RADIUS;

    float theta_i = (vR - vL) / ROBOT_WIDTH; //в этом вычислении опускается тангенс угла, тк tg(x) ~= x на малых углах
    Theta.tick(theta_i);

    v = (vR + vL) / 2;
    vX = v * cos(Theta.getOut());
    vY = v * sin(Theta.getOut());

    Distance.tick(v);
    X.tick(vX);
    Y.tick(vY);
}

void Odometry:: updateDir(Direction dir_)
{
    dir = dir_;
}

void Odometry::updateMazeCoords(Direction dir){
    mazeCoord.plusOrtVector(dir);
}

void Odometry::updateMazeCoords(Vec2 new_v){
    mazeCoord = new_v;
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

void Odometry::updateRelative()
{
    X_r = X;
    Y_r = Y;
    Theta_r = Theta;
    Distance_r = Distance;
}