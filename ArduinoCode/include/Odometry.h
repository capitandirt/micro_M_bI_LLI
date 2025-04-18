#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "Encoder.h"
#include "Maze.h"

class Integrator
{
private:
    float private_out = 0;
public:
    float out() {return private_out;}  
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
    int mazeCoordX, mazeCoordY;
    Direction dir;
public:
    
    float getX() {return X.out();}
    float getY() {return Y.out();}
    float getTheta() {return Theta.out();}
    float getDist() {return Distance.out();}
    int getMazeCoordX() {return mazeCoordX;}
    int getMazeCoordY() {return mazeCoordY;}
    Direction getDir() {return dir;}

    void update(float omegaL, float omegaR);
    void reset();
};

#endif // !_ODOMETRY_H_