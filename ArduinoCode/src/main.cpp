#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"

void setup()
{
	DEVICES::TEST::SET_SERIAL();
    DEVICES::INIT();

    //DEVICES::TEST::CONVERT_TO_SMART();
    //cycloStore.addSmart(SmartCycloAction_t::FWD_X, 1);
    cycloStore.addSmart(SmartCycloAction_t::SS90SR);
    cycloStore.addSmart(SmartCycloAction_t::SS90SR);
    cycloStore.addSmart(SmartCycloAction_t::SS90SR);
    cycloStore.addSmart(SmartCycloAction_t::SS90SR);
}

void loop(){
    static uint32_t last_time = 0;
    while(micros() - last_time < Ts_us)
        ;
    last_time = micros();
    DEVICES::TICK(last_time / 1000);

    //Serial.println(odometry.getTheta() * RAD_TO_DEG);
    cycloWorker.doCyclogram();
    // robot.statusHandler();
    cycloWorker.tryComplete();
}