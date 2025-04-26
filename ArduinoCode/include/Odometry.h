#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "Encoder.h"
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

private:
    float _out = 0;
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

    Vec2 mazeCoord;
    Direction dir = START_DIRECTION;

public:
    float getX() const noexcept;
    float getY() const noexcept;
    float getTheta() const noexcept;
    float getDist() const noexcept;
    Vec2 getMazeCoord() const noexcept;
    Direction getDir() const noexcept;

    void update(float omegaL, float omegaR) noexcept;
    void updateDir(Direction dir) noexcept;
    void reset() noexcept;
};

#endif // !_ODOMETRY_H_