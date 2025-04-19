#include "OptocouplerSensors.h"

void OptocouplerSensors::update()
{
    
}

Sense_t OptocouplerSensors::getSense()
{
    return sense.get();
}

Cell OptocouplerSensors::getCellFromSensors(Direction robotDir)
{
    //дописать опознавание стенок по датчикам
}