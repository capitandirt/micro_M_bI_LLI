#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"

void setup()
{
	DEVICES::TEST::SET_SERIAL();
    DEVICES::INIT();

    cycloStore.addSmart(SmartCycloAction_t::SS90ER);
}


void loop(){
    static uint32_t last_time = 0;
    while(micros() - last_time < Ts_us)
        ;
    DEVICES::TICK(last_time / 1000);
    
    last_time = micros();
    optocoupler.printSense();

    cycloWorker.doCyclogram();
    // robot.statusHandler();
    cycloWorker.tryComplete();
}