#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"

void setup()
{
    DEVICES::TEST::SET_SERIAL();
    DEVICES::INIT();

    delay(25); // ставлю delay чтобы датчики успели прочитать значение хотя бы раз
    optocoupler.setStaticError();

    // cycloStore.addSmart(SmartCycloAction_t::IP90L);
    // cycloStore.addSmart(SmartCycloAction_t::DELAY_025S, 8);
    // cycloStore.addSmart(SmartCycloAction_t::IP90R);
    // cycloStore.addSmart(SmartCycloAction_t::DELAY_025S, 8);
    // cycloStore.addSmart(SmartCycloAction_t::IP180);
    // cycloStore.addSmart(SmartCycloAction_t::DELAY_025S, 4);
    // cycloStore.addSmart(SmartCycloAction_t::SS90EL);
    // cycloStore.addSmart(SmartCycloAction_t::DELAY_025S, 4);
    // cycloStore.addSmart(SmartCycloAction_t::SS90ER);
}

void loop(){
    static uint32_t last_time = 0;
    while(micros() - last_time < Ts_us)
        ;
    last_time = micros();
    
    DEVICES::TICK(last_time / 1000);

    // Serial.println((int)programStatusSelector.getStatus());

    cycloWorker.doCyclogram();
    robot.stateMachine();
    cycloWorker.tryComplete();
}