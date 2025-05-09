#ifndef _CYCLO_STRUCTS_H_
#define _CYCLO_STRUCTS_H_

#include "Arduino.h"
#include "Odometry.h"
#include "OptocouplerSensors.h"

enum class SmartCycloAction_t : uint8_t{
    IDLE = 0,
    FWD1,
    FWD2,
    FWD3,
    FWD4,
    FWD5,
    FWD6,
    FWD7,
    FWD8,
    FWD9,
    FWD10,
    FWD11,
    FWD12,
    FWD13,
    FWD14,
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

    STOP,
};

enum class PrimitiveCycloAction_t : uint8_t{
    FORWARD = 0,
    
    LEFT = 1,
    BACK = 2,
    RIGHT = 3,
    
    STOP = 4,
    START = 5,
    BLANK = 6
};

struct RawCycloActionStore{
    PrimitiveCycloAction_t primitive : 3;
    SmartCycloAction_t smart : 5;
};

inline constexpr uint8_t toInt(SmartCycloAction_t sca){
    return static_cast<uint8_t>(sca);
}

inline constexpr uint8_t toInt(PrimitiveCycloAction_t pca){
    return static_cast<uint8_t>(pca);
}

inline const char* Str_SmartCyclogramAction[]{
    "IDLE",
    "FWD_HALF",
    "FWD1",
    "FWD2",
    "FWD3",
    "FWD4",
    "FWD5",
    "FWD6",
    "FWD7",
    "FWD8",
    "FWD9",
    "FWD10",
    "FWD11",
    "FWD12",
    "FWD13",
    "FWD14",
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
    "IP90R",
    "STOP"
};

struct Sensors
{
    float time;
    Odometry* robotState;
    OptocouplerSensors* optocoupler;
};

struct MotionStates
{
    float v_f0;
    float theta_i0;
    bool isComplete;
};

typedef void (*Cyclogram)(MotionStates*, Sensors*);
#define CYCLOGRAM(name) inline void name(MotionStates* ms, Sensors* s)

#endif // !_CYCLO_STRUCTS_H_