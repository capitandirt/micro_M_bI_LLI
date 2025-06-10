#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"


void setup()
{
    DEVICES::TEST::SET_SERIAL();
    DEVICES::INIT();

    for(uint16_t i = 0; i < 1000; i++){
        Serial.print(0);
    }
    // DEVICES::TEST::CONVERT_TO_SMART();
    // while(true);

    // cycloStore.addSmart(SmartCycloAction_t::SS90EL);
    // cycloStore.addSmart(SmartCycloAction_t::SS90ER);
    // cycloStore.addSmart(SmartCycloAction_t::SS90EL);

    // cycloStore.addSmart(SmartCycloAction_t::FWD_HALF);
    // cycloStore.addSmart(SmartCycloAction_t::SS90EL);

    // cycloStore.addSmart(SmartCycloAction_t::TO_BACK_ALIGN);
    // cycloStore.addSmart(SmartCycloAction_t::FROM_BACK_ALIGN_TO_CENTER);
    // cycloStore.addSmart(SmartCycloAction_t::FWD_X, 2);
    // cycloStore.addSmart(SmartCycloAction_t::SS90SL);
    // cycloStore.addSmart(SmartCycloAction_t::SD45SR);
    // cycloStore.addSmart(SmartCycloAction_t::DS45SL);
    cycloStore.addSmart(SmartCycloAction_t::SS180SR);
    // cycloStore.addSmart(SmartCycloAction_t::FWD_X);
    // cycloStore.addSmart(SmartCycloAction_t::STOP);

    // cycloStore.addSmart(SmartCycloAction_t::IP90R);
    // cycloStore.addSmart(SmartCycloAction_t::IP90L);
    // cycloStore.addSmart(SmartCycloAction_t::IP180);

    delay(25); // ставлю delay чтобы датчики успели прочитать значение хотя бы раз
    optocoupler.setStaticError();
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