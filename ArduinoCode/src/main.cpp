#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"

void setup()
{
    DEVICES::INIT();
	DEVICES::TEST::SET_SERIAL();

    // Direction d = Direction::S;
    // actionsHandler.primitivesToExplorers(Direction::S, {0, 0}, Direction::N, {0, 0});

    // DEVICES::TEST::CYCLOGRAMS();
    // DEVICES::TEST::BFS();
    // DEVICES::TEST::CONVERT_PATH_TO_CYCLOGRAMS();
}

void loop(){
    // DEVICES::TEST::OPTOCOUPLER();
    // cycloStore.printSmarts();

    // Serial.println((int)odometry.getDir()); 
}
//программа выполняет функцию tick через прерывание по таймеру каждые Ts_s секунд