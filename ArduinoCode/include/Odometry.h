#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "Drivers/Encoder.h"
#include "CellsTypes.h"

class Integrator
{
public:
    float getOut() const noexcept {
        return _out;
    }  

    void tick(float in)
    {
        _out += in * Ts_s;
    }

    void reset()
    {
        _out = 0;
    }

    float operator-(const Integrator& other) const{
        return _out - other._out;
    }

    void operator=(const float val){
        _out = val;
    }

    void operator=(const Integrator& other){
        _out = other._out;
    }

private:
    float _out = 0;
};

class Odometry
{
private:
    Integrator X, Y, Theta, Distance;
    Integrator X_r, Y_r, Theta_r, Distance_r;

    float vL = 0, vR = 0, vX = 0, vY = 0, v = 0;

    mutable Vec2 mazeCoord = START_ROBOT_COORDS;
    Direction dir = START_ROBOT_DIRECTION;

public:
    float getX() const noexcept;
    float getY() const noexcept;
    float getTheta() const noexcept;
    float getDist() const noexcept;

    float getRelativeX() const noexcept;
    float getRelativeY() const noexcept;
    float getRelativeTheta() const noexcept;
    float getRelativeDist() const noexcept;

    void reset() noexcept;
    void updateRelative() noexcept;

    Vec2 getMazeCoords() const noexcept;
    Direction getDir() const noexcept;

    void printDir() const noexcept;

    void update(float omegaL, float omegaR) noexcept;
    void updateDir(Direction dir_) noexcept;
    void updateMazeCoords(Direction dir);
    void updateMazeCoords(Vec2 new_v);
};

#endif // !_ODOMETRY_H_