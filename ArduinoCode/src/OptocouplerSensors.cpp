#include "OptocouplerSensors.h"

void OptocouplerSensors::update()
{
    
}

void OptocouplerSensors::getDist(float* dist)
{
    dist[0] = _dist[0];
    dist[1] = _dist[1];
    dist[2] = _dist[2]; 
    dist[3] = _dist[3];
}

Cell OptocouplerSensors::getCellFromSensors(Direction robotDir)
{
    //дописать опознавание стенок по датчикам
}