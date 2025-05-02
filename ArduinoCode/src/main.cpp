#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"

void setup()
{
    DEVICES::INIT();
	DEVICES::TEST::SET_SERIAL();
    // DEVICES::TEST::EXPLORER_CYC();
    // DEVICES::TEST::CYCLOGRAMS();
    // DEVICES::TEST::BFS();
    // DEVICES::TEST::CONVERT_PATH_TO_CYCLOGRAMS();
}

uint32_t last_time = 0;

void loop(){
    while(micros() - last_time < Ts_us)
        ;
    last_time = micros();

    DEVICES::TEST::OPTOCOUPLERS();
    // DEVICES::TICK();
    // cycloStore.printSmarts();
}
//программа выполняет функцию tick через прерывание по таймеру каждые Ts_s секунд