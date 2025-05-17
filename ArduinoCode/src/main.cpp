#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"

void setup()
{
	DEVICES::TEST::SET_SERIAL();
    DEVICES::INIT();    

    // DEVICES::TEST::CONVERT_TO_SMART();
    //DEVICES::TEST::EXPLORER_LEFT_RIGHT_SMARTS();
    // DEVICES::TEST::UNDEF_CELL_WALLS();
    // DEVICES::TEST::FWDE();
    // DEVICES::TEST::EXPLORER_LEFT_RIGHT_SMARTS();
    // DEVICES::TEST::CYCLOGRAMS();
    // DEVICES::TEST::BFS();
    // DEVICES::TEST::CONVERT_PATH_TO_CYCLOGRAMS();
}


uint32_t last_time = 0;
void loop(){
    while(micros() - last_time < Ts_us)
        ;
    last_time = micros();

    DEVICES::TICK(last_time / 1000);
}