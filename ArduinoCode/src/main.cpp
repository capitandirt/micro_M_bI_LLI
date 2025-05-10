#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"

void setup()
{
    DEVICES::INIT();

	DEVICES::TEST::SET_SERIAL();
    // DEVICES::TEST::UNDEF_CELL_WALLS();
    // DEVICES::TEST::FWD_3X();
    // DEVICES::TEST::EXPLORER_LEFT_RIGHT_SMARTS();
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

    // cycloStore.printSmarts();
    // Serial.print(odometry.getMazeCoords().x);
    // Serial.print(" ");
    // Serial.print(odometry.getMazeCoords().y);
    // Serial.print(" ");
    // Serial.println((int)maze.GetPathDir(0));
    // maze.PrintDirPath();
    // cycloStore.printSmarts();
    // DEVICES::TEST::OPTOCOUPLERS_SENSE();
    // DEVICES::TEST::OPTOCOUPLERS_MASK();
    DEVICES::TICK();
}