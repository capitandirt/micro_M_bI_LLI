#include "OptocouplerSensors.h"

void OptocouplerSensors::init(){
    pinMode(EMITERS_FWD, OUTPUT);
    pinMode(EMITERS_SIDE, OUTPUT);
    pinMode(SENSOR_0, INPUT);
    pinMode(SENSOR_1, INPUT);
    pinMode(SENSOR_2, INPUT);
    pinMode(SENSOR_3, INPUT);
}

void OptocouplerSensors::tick(){
    digitalWrite(EMITERS_SIDE, 1);
    sense[OptocouplerSense::From::LEFT] = analogRead(SENSOR_0);
    sense[OptocouplerSense::From::RIGHT] = analogRead(SENSOR_1);
    digitalWrite(EMITERS_SIDE, 0);

    digitalWrite(EMITERS_FWD, 1);
    sense[OptocouplerSense::From::FORWARD_L] = analogRead(SENSOR_2);
    sense[OptocouplerSense::From::FORWARD_R] = analogRead(SENSOR_3);
    digitalWrite(EMITERS_SIDE, 0);
}

Sense_t OptocouplerSensors::getSense(){
    return sense.get();
}

Cell OptocouplerSensors::getCellFromSensors(Direction robotDir){
    
}