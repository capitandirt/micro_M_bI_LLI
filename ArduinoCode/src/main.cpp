#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"


void setup()
{
    DEVICES::TEST::SET_SERIAL();
    DEVICES::INIT();

    cycloStore.addSmart(SmartCycloAction_t::SS90SL);

    // cycloStore.addSmart(SmartCycloAction_t::SD135SL);
    // cycloStore.addSmart(SmartCycloAction_t::SD45SL);
    // cycloStore.addSmart(SmartCycloAction_t::DIAG_X, 3);
    // cycloStore.addSmart(SmartCycloAction_t::DS45SL);
    // cycloStore.addSmart(SmartCycloAction_t::FWD_X, 6);

    delay(25); // ставлю delay чтобы датчики успели прочитать значение хотя бы раз
    optocoupler.setStaticError(optocoupler.getSense().left, optocoupler.getSense().right);
    //optocoupler.setStaticError(0, 0);
}

void loop(){
    static uint32_t last_time = 0;
    while(micros() - last_time < Ts_us)
        ;
    last_time = micros();
    
    DEVICES::TICK(last_time / 1000);

    // gyro.printYaw();
    
    cycloWorker.doCyclogram();
    // // robot.stateMachine();
    cycloWorker.tryComplete();

    // Serial.println(micros() - last_time);
}