#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"


void setup()
{
    DEVICES::TEST::SET_SERIAL();
    DEVICES::INIT();

    cycloStore.addSmart(SmartCycloAction_t::TO_BACK_ALIGN);
    cycloStore.addSmart(SmartCycloAction_t::FROM_BACK_ALIGN_TO_CENTER);
    cycloStore.addSmart(SmartCycloAction_t::SD135SL);
    cycloStore.addSmart(SmartCycloAction_t::DD90SR);
    cycloStore.addSmart(SmartCycloAction_t::DIAG_X, 3);
    cycloStore.addSmart(SmartCycloAction_t::DS135SR);
    cycloStore.addSmart(SmartCycloAction_t::SS180SL);
    cycloStore.addSmart(SmartCycloAction_t::SS180SR);
    cycloStore.addSmart(SmartCycloAction_t::FWD_X);
    cycloStore.addSmart(SmartCycloAction_t::DIAG_X);
    cycloStore.addSmart(SmartCycloAction_t::FWD_X);
    // cycloStore.addSmart(SmartCycloAction_t::SS90SL);
    // cycloStore.addSmart(SmartCycloAction_t::FWD_X, 3);
    cycloStore.addSmart(SmartCycloAction_t::STOP);

    delay(25); // ставлю delay чтобы датчики успели прочитать значение хотя бы раз
    optocoupler.setStaticError();
    //optocoupler.setStaticError(0, 0);
}

void loop(){
    static uint32_t last_time = 0;
    while(micros() - last_time < Ts_us)
        ;
    last_time = micros();
    
    DEVICES::TICK(last_time / 1000);

    //Serial.println(odometry.getTheta());

    // if(abs(odometry.getTheta() - HALF_PI) < 0.05){
    //     indicator.on();
    // }
    // else indicator.off();
    cycloWorker.doCyclogram();
    // // robot.stateMachine();
    cycloWorker.tryComplete();

    // Serial.println(micros() - last_time);
}