#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"

void setup()
{
    // DEVICES::INIT();
	DEVICES::TEST::SET_SERIAL();

    Direction d = Direction::S;
    Vec2 v = {0, 0};
    actionsHandler.primitivesToExplorers(Direction::W, {0, 0}, v);

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