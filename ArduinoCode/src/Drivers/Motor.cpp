#include "Drivers/Motor.h"

void Motor::init(){
    pinMode(DIR, OUTPUT);
    pinMode(PWM, OUTPUT);
}

void Motor::drive(float u){
    int16_t pwm = constrain(MAX_PMW * u / 9, -MAX_PMW, MAX_PMW);

    if (pwm >= 0){
        digitalWrite(DIR, M_POLARITY);
        analogWrite(PWM, pwm);
    }
    else{
        digitalWrite(DIR, !M_POLARITY);
        analogWrite(PWM, -pwm);
    }
}