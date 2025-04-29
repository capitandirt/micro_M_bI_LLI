#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"

void setup()
{
    // DEVICES::INIT();
    
	DEVICES::TEST::SET_SERIAL();
    // DEVICES::TEST::CYCLOGRAMS();
    // DEVICES::TEST::BFS();
    DEVICES::TEST::CONVERT_PATH_TO_CYCLOGRAMS();

    // optocoupler.init();
}

void loop(){}
