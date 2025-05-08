#include "Motor.h"

void Motor::init(){
    pinMode(DIR, OUTPUT);
    pinMode(PWM, OUTPUT);
}

void Motor::drive(float u){
    int16_t pwm = constrain(255.0 * u / V_BATT, -255, 255);
    if (pwm >= 0){
        digitalWrite(DIR, M_POLARITY);
        analogWrite(PWM, pwm);
    }
    else{
        digitalWrite(DIR, !M_POLARITY);
        analogWrite(PWM, -pwm);
    }
}