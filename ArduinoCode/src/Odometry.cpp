#include "Odometry.h"

#if USE_GYRO
Odometry::Odometry(Gyro* gyro_)
{
    gyro = gyro_;
}
#endif


float Odometry::getX() const{
    return X.getOut();
}

float Odometry::getY() const{
    return Y.getOut();
}

float Odometry::getTheta() const{
    // #if USE_GYRO
    // return gyro->getYawAngle();
    // #else
    // return ThetaIntegrator.getOut();
    // #endif

    return circle_mod(Theta.getOut());
}

void Odometry::setTheta(float theta_) noexcept{
    #if USE_GYRO
    gyro->modifyYawOffset(circle_mod(theta_ - gyro->getYawAngle()));
    Theta = circle_mod(theta_);
    #else
    ThetaIntegrator = theta_;
    #endif
}

void Odometry::setRelativeDist(float dist){
    Distance_r = dist;
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
    #if USE_GYRO
    return circle_mod(Theta - Theta_r);  
    #else
    return ThetaIntegrator - Theta_r;
    #endif
}

float Odometry::getRelativeDist() const{
    return Distance_r.getOut();
}

Vec2 Odometry::getMazeCoords() const{
    return mazeCoords;
}

Direction Odometry::getDir() const{
    return dir;
}

Direction Odometry::getStartFastDir() const{
    return startFastDir;
}

void Odometry::setDir(Direction dir_) noexcept{
    dir = dir_;
}

void Odometry::printMazeCoords() const{
    PRINT(mazeCoords.x);
    PRINT(" ");
    PRINTLN(mazeCoords.y);
}

void Odometry::printDir() const{
    switch (dir)
    {
    case Direction::N:
        PRINTLN("N");
        break;
    case Direction::S:
        PRINTLN("S");
        break;
    case Direction::E:
        PRINTLN("E");
        break;
    case Direction::W:
        PRINTLN("W");
        break;
    }
}

void Odometry::tick(float omegaL, float omegaR)
{
    vL = omegaL * WHEEL_RADIUS;
    vR = omegaR * WHEEL_RADIUS;

    const float theta_i = (vR - vL) / ROBOT_WIDTH; //в этом вычислении опускается тангенс угла, тк tg(x) ~= x на малых углах

    v = (vR + vL) / 2;

    // #if USE_GYRO
    vX = v * cos(Theta.getOut());
    vY = v * sin(Theta.getOut());
    // #else

    // vX = v * cos(ThetaIntegrator.getOut());
    // vY = v * sin(ThetaIntegrator.getOut());
    // #endif

    if(gyro->isNewYaw()){
        Theta = gyro->getYawAngle();
    }
    else{
        Theta.tick(theta_i);
    }

    Distance.tick(v);
    Distance_r.tick(v);
    X.tick(vX);
    Y.tick(vY);
}

void Odometry::updateMazeCoords(Direction dir){
    mazeCoords.plusOrtVector(dir);
}

void Odometry::updateMazeCoords(Vec2 new_v){
    mazeCoords = new_v;
}

void Odometry::reset()
{
    vL = 0;
    vR = 0;
    vX = 0;
    vY = 0;
    v = 0;
    mazeCoords = {0, 0};
    dir = Direction::N;
    X.reset();
    Y.reset();

    gyro->setYawOffset(gyro->getYawAngle());
    Theta = gyro->getYawAngle();    

    Distance.reset();
}

void Odometry::updateRelative()
{
    X_r = X;
    Y_r = Y;

    // #if USE_GYRO
    Theta_r = Theta;
    // #else
    // #endif

    Distance_r = 0;
}