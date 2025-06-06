#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "Drivers/Encoder.h"
#include "CellsTypes.h"

class Integrator
{
public:
    Integrator(float val = 0) : _out(val) {} 

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

    void operator=(const double val){
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

    mutable Vec2 mazeCoords = START_ROBOT_COORDS;
    Direction dir;
    Direction startFastDir;

public:
    float getX() const;
    float getY() const;
    float getTheta() const;
    float setTheta(float theta_) noexcept;
    float getDist() const;

    float getRelativeTheta() const;
    float getRelativeDist() const;
    float getRelativeX() const;
    float getRelativeY() const;

    void reset() noexcept;
    void updateRelative() noexcept;

    Vec2 getMazeCoords() const noexcept;
    Direction getDir() const noexcept;
    Direction getStartFastDir() const noexcept;
    void setDir(Direction dir_) noexcept;

    void printMazeCoords() const noexcept;
    void printDir() const noexcept;

    void tick(float omegaL, float omegaR) noexcept;
    void updateMazeCoords(Direction dir);
    void updateMazeCoords(Vec2 new_v);
};

#endif // !_ODOMETRY_H_