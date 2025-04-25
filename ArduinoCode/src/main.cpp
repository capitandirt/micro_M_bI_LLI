#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"

void setup()
{
	DEVICES::TEST::SET_SERIAL();

    // DEVICES::TEST::CYCLOGRAMS();
    // DEVICES::INIT();
    // DEVICES::TEST::BFS();
    DEVICES::TEST::CONVERT_PATH_TO_CYCLOGRAMS();
}

void loop(){
    // static uint32_t timer = micros();
    // while(micros() - timer < Ts_us)
    //   ;
    // timer = micros();
    // // Serial.print("yes");
    // DEVICES::TICK(); 
}
