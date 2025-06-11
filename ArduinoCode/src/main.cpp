#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"


void setup()
{
    DEVICES::TEST::SET_SERIAL();
    DEVICES::INIT();

    

    // DEVICES::TEST::CONVERT_TO_SMART();
    // while(true);

    // cycloStore.addSmart(SmartCycloAction_t::SD135SL);
    // cycloStore.addSmart(SmartCycloAction_t::DS135SR);
    // cycloStore.addSmart(SmartCycloAction_t::SD135SR);
    // cycloStore.addSmart(SmartCycloAction_t::DS135SL);

    delay(25); // ставлю delay чтобы датчики успели прочитать значение хотя бы раз
    optocoupler.setStaticError();

    cycloStore.addSmart(SmartCycloAction_t::SS90SR);
}

void loop(){
    static uint32_t last_time = 0;
    while(micros() - last_time < Ts_us)
        ;
    last_time = micros();
    
    DEVICES::TICK(last_time / 1000);

    // Serial.println("main: " + String(odometry.getTheta() * RAD_TO_DEG)); 

    // if(abs(odometry.getTheta() - HALF_PI) < 0.05){
    //     indicator.on();
    // }
    // else indicator.off();

    // optocoupler.printMask();

    cycloWorker.doCyclogram();
    // robot.stateMachine();
    cycloWorker.tryComplete();

    // Serial.println(micros() - last_time);
}