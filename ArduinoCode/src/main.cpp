#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"

void setup()
{
    DEVICES::TEST::SET_SERIAL();
    DEVICES::INIT();

    delay(25); // ставлю delay чтобы датчики успели прочитать значение хотя бы раз
    optocoupler.setStaticError();
}

void loop(){
    static uint32_t last_time = 0;
    while(micros() - last_time < Ts_us)
        ;
    last_time = micros();
    
    DEVICES::TICK(last_time / 1000);

    cycloWorker.doCyclogram();
    robot.stateMachine();
    cycloWorker.tryComplete();
}