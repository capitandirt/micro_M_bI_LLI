#include <Arduino.h>
#include "Devices.h"
#include "DevicesMethods.h"


void setup()
{
    DEVICES::TEST::SET_SERIAL();
    DEVICES::INIT();

    // DEVICES::TEST::CONVERT_TO_SMART();
    // while(true);

    delay(25); // ставлю delay чтобы датчики успели прочитать значение хотя бы раз
    optocoupler.setStaticError();

    // DEVICES::TEST::CONVERT_TO_SMART();

    // while(true)
}

void loop(){
    static uint32_t last_time = 0;
    while(micros() - last_time < Ts_us)
        ;
    last_time = micros();

    DEVICES::TICK(last_time / 1000);

    // Serial.print(functionalSelector.isLever(0));
    // Serial.print(functionalSelector.isLever(1));
    // Serial.print(functionalSelector.isLever(2));
    // Serial.println(functionalSelector.isLever(3));

    cycloWorker.doCyclogram();
    robot.stateMachine();
    cycloWorker.tryComplete();
}