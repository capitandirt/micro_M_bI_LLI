#ifndef _CYCLO_STRUCTS_H_
#define _CYCLO_STRUCTS_H_

#include "Arduino.h"
#include "Odometry.h"

enum class SmartCycloAction_t : uint8_t{
    STOP = 0,
    IDLE,
    FWD,
    FWD_HALF,
    SS90SL,
    SS90EL,
    SS90SR,
    SS90ER,
    SD45SL,
    SD45SR,
    DS45SL,
    DS45SR,
    SD135SL,
    SD135SR,
    SS180L,
    SS180R,
    IP180,
    IP90L,
    IP90R,

    CYCLO_ACTION_SIZE
};

constexpr char* Str_SmartCyclogramAction[]{
    "STOP",
    "IDLE",
    "FWD",
    "FWD_HALF",
    "SS90SL",
    "SS90EL",
    "SS90SR",
    "SS90ER",
    "SD45SL",
    "SD45SR",
    "DS45SL",
    "DS45SR",
    "SD135SL",
    "SD135SR",
    "SS180L",
    "SS180R",
    "IP180",
    "IP90L",
    "IP90R"
};

enum class PrimitiveCycloAction_t : uint8_t{
    FORWARD = 0,
    
    LEFT = 1,
    RIGHT = 3,
    
    STOP = 2,
    BLANK = 4
};

struct Sensors
{
    float time;
    Odometry* robotState;
};

struct MotionStates
{
    float v_f0;
    float theta_i0;
    bool isComplete;
};

typedef void (*CycloAction)(MotionStates*, Sensors*);


#endif // !_CYCLO_STRUCTS_H_