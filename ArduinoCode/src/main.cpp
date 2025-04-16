#include <Arduino.h>
#include "Config.h"
#include "Devices.h"

void setup()
{
    // DEVICES::INIT();
    // DEVICES::TEST::CYCLOGRAMS();
    // DEVICES::TEST::BFS();
    DEVICES::TEST::CONVERT_PATH_TO_CYCLOGRAMS();
}

void loop(){
    // static uint32_t timer = micros();
    // while(micros() - timer < Ts_us)
    //   ;
    // timer = micros();
    // DEVICES::TICK(); 
}
