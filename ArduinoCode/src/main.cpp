#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"

void setup()
{
    DEVICES::TEST::SET_SERIAL();
    DEVICES::INIT();
    // DEVICES::TEST::PRIM_TO_FAST();
    cycloStore.addSmart(SmartCycloAction_t::DIAG_X, 1);
    // while(true);
}

void loop(){
    static uint32_t last_time = 0;
    while(micros() - last_time < Ts_us)
        ;
    last_time = micros();

    DEVICES::TICK(last_time / 1000);

    cycloWorker.doCyclogram();
    //robot.statusHandler();
    cycloWorker.tryComplete();
}