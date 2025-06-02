#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "Arduino.h"
#include "Config.h"
#include "Battery.h"

struct MotorConnectionParams{
    const uint8_t
        DIR, 
        PWM,
        M_POLARITY;
    Battery* const _battery;
};

class Motor : public MotorConnectionParams{
public:
    Motor(MotorConnectionParams* mcp) : MotorConnectionParams(*mcp){}
    void init();
    void drive(float u);

private:
    static constexpr float MAX_PMW = 255;
};

#endif // !_MOTOR_H_