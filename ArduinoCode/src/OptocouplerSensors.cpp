#include "OptocouplerSensors.h"

void OptocouplerSensors::init(){
    pinMode(EMITERS_A, OUTPUT);
    pinMode(EMITERS_B, OUTPUT);
}

void OptocouplerSensors::update(){
    digitalWrite(EMITERS_A, 1);
    digitalWrite(EMITERS_B, 1);

    Serial.print(analogRead(SENSOR_0));
    Serial.print(' ');
    Serial.print(analogRead(SENSOR_1));
    Serial.print(' ');
    Serial.print(analogRead(SENSOR_2));
    Serial.print(' ');
    Serial.print(analogRead(SENSOR_3));
    Serial.println(' ');
}

Sense_t OptocouplerSensors::getSense(){
    return sense.get();
}

Cell OptocouplerSensors::getCellFromSensors(Direction robotDir){
    
}