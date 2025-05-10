#ifndef _CYCLO_STRUCTS_H_
#define _CYCLO_STRUCTS_H_

#include "Arduino.h"
#include "Odometry.h"
#include "OptocouplerSensors.h"

enum class SmartCycloAction_t : uint8_t{
    IDLE = 0,
    CLUSTER_DOT,
    FWD_HALF,
    FWD,
    SS90EL,
    SS90ER,
    SS90SL,
    SS90SR,
    SD45SL,
    SD45SR,
    DS45SL,
    DS45SR,
    SD135SL,
    SD135SR,
    DS135SL,
    DS135SR,
    SS180SL,
    SS180SR,
    IP180,
    IP90L,
    IP90R,
    TO_ALIGN,
    FROM_ALIGN_TO_CENTER,

    STOP // <! END OF ENUM
};

enum class PrimitiveCycloAction_t : uint8_t{ // LEFT, RIGHT, FORWARD, BACK, STOP
    FORWARD = 0,
    
    LEFT = 1,
    BACK = 2, // не встречается в итоговом пути, не обрабатывается в convertToSmart
    RIGHT = 3,
    
    STOP = 4,
    BLANK = 5
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
    "CLUSTER_DOT",
    "FWD_HALF",
    "FWD",
    "SS90EL",
    "SS90ER",
    "SS90SL",
    "SS90SR",
    "SD45SL",
    "SD45SR",
    "DS45SL",
    "DS45SR",
    "SD135SL",
    "SD135SR",
    "DS135SL",
    "DS135SR",
    "SS180SL",
    "SS180SR",
    "IP180",
    "IP90L",
    "IP90R",
    "TO_ALIGN",
    "FROM_ALIGN_TO_CENTER",

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

using Cyclogram = void (*)(MotionStates*, Sensors*);
#define CYCLOGRAM(name) inline void name(MotionStates* ms, Sensors* s)

#endif // !_CYCLO_STRUCTS_H_