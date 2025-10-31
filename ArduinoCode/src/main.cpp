#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"

void setup()
{
    DEVICES::TEST::SET_SERIAL();
    DEVICES::INIT();


    delay(25); // ставлю delay чтобы датчики успели прочитать значение хотя бы раз
    optocoupler.setStaticError();
    // cycloStore.addSmart(SmartCycloAction_t::IP90L, 10);
    // cycloStore.addSmart(SmartCycloAction_t::IP90R, 1);
    // cycloStore.addSmart(SmartCycloAction_t::IP180, 1);
    // cycloStore.addSmart(SmartCycloAction_t::DELAY_025S, 10);
    // cycloStore.addSmart(SmartCycloAction_t::SS180SL);
    // DEVICES::TEST::CONVERT_TO_SMART();
    
}

void loop(){
    static uint32_t last_time = 0;
    while(micros() - last_time < Ts_us)
        ;
    last_time = micros();

    DEVICES::TICK(last_time / 1000);    

    cycloWorker.doCyclogram();
    robot.stateMachine();
    cycloWorker.tryComplete();
}