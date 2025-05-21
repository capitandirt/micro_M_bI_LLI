#ifndef _CYCLO_STRUCTS_H_
#define _CYCLO_STRUCTS_H_

#include "Arduino.h"
#include "Odometry.h"
#include "Drivers/OptocouplerSensors.h"

enum class SmartCycloAction_t : uint8_t{
    IDLE = 0,
    CLUSTER_DOT,
    FWD_HALF,
    FWDE,
    FWD_X,
    DIAG_X,
    SS90EL,
    SS90ER,
    SS90SL,
    SS90SR,
    DD90SL,
    DD90SR,
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
    IP90L,
    IP90R,
    TO_BACK_ALIGN,
    FROM_BACK_ALIGN_TO_CENTER,
    TO_FORWARD_ALIGN,
    FROM_FORWARD_ALIGN_TO_CENTER,

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

constexpr uint8_t BYTE_SIZE = 8;
constexpr uint8_t VALUE_SMART_BITS_IN_RAW_STORE = 5;
constexpr uint8_t VALUE_PRIMITIVE_BITS_IN_RAW_STORE = BYTE_SIZE - VALUE_SMART_BITS_IN_RAW_STORE;

enum class X_t : uint8_t{ NONE = 1, LAST = (1 << VALUE_SMART_BITS_IN_RAW_STORE) - 1};

struct SmartSubmission{
    SmartCycloAction_t smart;
    X_t x;
};

struct RawCycloActionStore{    
    // smart can contain X_t too
    SmartCycloAction_t smart : VALUE_SMART_BITS_IN_RAW_STORE;
    PrimitiveCycloAction_t primitive : VALUE_PRIMITIVE_BITS_IN_RAW_STORE; // stupid warning
};
inline constexpr uint8_t toInt(const SmartCycloAction_t sca){
    return static_cast<uint8_t>(sca);
}

inline constexpr uint8_t toInt(const PrimitiveCycloAction_t pca){
    return static_cast<uint8_t>(pca);
}

inline bool isOppositeTurns(const PrimitiveCycloAction_t TURN1, const PrimitiveCycloAction_t TURN2)
{
    return (toInt(TURN1) + DIRECTION_SIZE/2) % DIRECTION_SIZE == toInt(TURN2);
}

inline PrimitiveCycloAction_t toOpposite(const PrimitiveCycloAction_t TURN)
{
    if(TURN == PrimitiveCycloAction_t::LEFT) return PrimitiveCycloAction_t::RIGHT;
    else if(TURN == PrimitiveCycloAction_t::RIGHT) return PrimitiveCycloAction_t::LEFT;
    else return PrimitiveCycloAction_t::BLANK;
}

inline constexpr uint8_t toInt(const X_t x){
    return static_cast<uint8_t>(x);
}

inline X_t toX_t(const uint8_t val){
    if (val == 0) return X_t::NONE;
    if (val > toInt(X_t::LAST)) return X_t::LAST;

    return static_cast<X_t>(val);
}

inline X_t toX_t(const SmartCycloAction_t val){
    if(toInt(val) > toInt(X_t::LAST)) return X_t::LAST;
    if(toInt(val) == toInt(X_t::NONE)) return X_t::NONE;

    return static_cast<X_t>(toInt(val));
}

inline const char* Str_SmartCyclogramAction[]{
    "IDLE",
    "CLUSTER_DOT",
    "FWD_HALF",
    "FWDE",
    "FWD_",
    "DIAG_",
    "SS90EL",
    "SS90ER",
    "SS90SL",
    "SS90SR",
    "DD90SL",
    "DD90SR",
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
    "IP90L",
    "IP90R",
    "TO_BACK_ALIGN",
    "FROM_BACK_ALIGN_TO_CENTER",
    "TO_FORWARD_ALIGN",
    "FROM_FORWARD_ALIGN_TO_CENTER",

    "STOP"
};

struct Sensors
{
    float time;
    Odometry* odometry;
    OptocouplerSensors* optocoupler;
};

struct MotionStates
{
    float v_f0;
    float theta_i0;
    bool isComplete;
};

struct CycloContext{
    MotionStates ms;
    Sensors s;

    void reload(){
        ms.isComplete = 0;
        s.odometry->updateRelative();
    }
};

using Cyclogram = void (*)(MotionStates*, const Sensors*, uint8_t);
#define CYCLOGRAM(name) inline void name(MotionStates* ms, const Sensors* s, uint8_t x = toInt(X_t::NONE))

#endif // !_CYCLO_STRUCTS_H_