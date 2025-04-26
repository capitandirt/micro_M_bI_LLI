#ifndef _CYCLO_STORE_H_
#define _CYCLO_STORE_H_

#include "CycloActions.h"
#include "CycloTypes.h"

class CycloStore{
public:
    static constexpr uint8_t CYCLO_PROG_SIZE = 64;
    static constexpr uint8_t CYCLO_ACTION_SIZE = toInt(SmartCycloAction_t::STOP) + 1;

    CycloStore(){load_cyclograms();}

    void addSmart(SmartCycloAction_t action);
    void addPrimitive(PrimitiveCycloAction_t action);

    SmartCycloAction_t popFrontSmart();
    PrimitiveCycloAction_t popFrontPrimitive();
    
    PrimitiveCycloAction_t virtualPopFrontPrimitive();
    void virtualGoBack(uint8_t steps);
    void virtualPrimitiveRelease();

    bool smartIsEmpty();
    bool primitiveIsEmpty();

    void reloadSmarts();
    void reloadPrimitives();

    Cyclogram cyclogramFrom();

    void printSmarts() const;
    void printPrimitives() const;
    bool checkTranslationTo(String cyc) const;
private:
    void load_cyclograms();

private:
    uint8_t _smart_cyc_act_counter = 0;
    uint8_t _smart_cyc_act_end = 0;
      
    uint8_t _primitive_cyc_act_counter = 0;
    uint8_t _primitive_cyc_act_end = 0;

    uint8_t _virtual_primitive_cyc_act_counter = 0;

    RawCycloActionStore _cyclo_program[CYCLO_PROG_SIZE];  
    Cyclogram _cyclograms[CYCLO_ACTION_SIZE];
};

#endif // !_CYCLO_STORE_H_