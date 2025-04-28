#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"

void setup()
{
	DEVICES::TEST::SET_SERIAL();

    // DEVICES::TEST::CYCLOGRAMS();
    DEVICES::INIT();
    // DEVICES::TEST::BFS();
    // DEVICES::TEST::CONVERT_PATH_TO_CYCLOGRAMS();

    // optocoupler.init();
}

void loop(){
    Sense_t sense = optocoupler.getSense();
    Serial.print(sense.forward_l);
    Serial.print(' ');
    Serial.print(sense.forward_r);
    Serial.print(' ');
    Serial.print(sense.left);
    Serial.print(' ');
    Serial.println(sense.right);
}
