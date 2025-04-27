#include "CycloUtilits/CycloStore.h"

void CycloStore::addSmart(SmartCycloAction_t action){
    if(_smart_cyc_act_end < CYCLO_PROG_SIZE){
        _cyclo_program[_smart_cyc_act_end++].smart = action;
    }
}

void CycloStore::addPrimitive(PrimitiveCycloAction_t action){
    if(_primitive_cyc_act_counter < CYCLO_PROG_SIZE){
        _cyclo_program[_primitive_cyc_act_end++].primitive = action;
    }
}

SmartCycloAction_t CycloStore::popFrontSmart(){
    if(_smart_cyc_act_counter >= _smart_cyc_act_end) return SmartCycloAction_t::IDLE;
    
    return _cyclo_program[_smart_cyc_act_counter++].smart;
}

PrimitiveCycloAction_t CycloStore::popFrontPrimitive(){
    if(_primitive_cyc_act_counter >= _primitive_cyc_act_end) return PrimitiveCycloAction_t::BLANK;
    
    _virtual_primitive_cyc_act_counter++;
    return _cyclo_program[_primitive_cyc_act_counter++].primitive;
}

PrimitiveCycloAction_t CycloStore::virtualPopFrontPrimitive(){
    if(_virtual_primitive_cyc_act_counter >= _primitive_cyc_act_end) return PrimitiveCycloAction_t::BLANK;

    return _cyclo_program[_virtual_primitive_cyc_act_counter++].primitive;
}
void CycloStore::virtualGoBack(){
    _virtual_primitive_cyc_act_counter = _primitive_cyc_act_counter;
}

void CycloStore::virtualPrimitiveRelease(){
    _primitive_cyc_act_counter = _virtual_primitive_cyc_act_counter;
}

bool CycloStore::smartIsEmpty(){
    return _smart_cyc_act_counter >= _smart_cyc_act_end - 1;
}

bool CycloStore::primitiveIsEmpty(){
    return _primitive_cyc_act_counter >= _primitive_cyc_act_end - 1;
}

void CycloStore::reloadSmarts(){
    _smart_cyc_act_end = 0;
    _smart_cyc_act_counter = 0;
}

void CycloStore::reloadPrimitives(){
    _primitive_cyc_act_end = 0;
    _primitive_cyc_act_counter = 0;
}
 
Cyclogram CycloStore::cyclogramFrom(){
    return _cyclograms[toInt(popFrontSmart())];
}

void CycloStore::printSmarts() const{
    Serial.println("Smarts");
    for(uint8_t i = 0; i < _smart_cyc_act_end; i++){
        Serial.print(Str_SmartCyclogramAction[toInt(_cyclo_program[i].smart)]);
        // Serial.print(toInt(_cyclo_program[i].smart));
        Serial.print(' ');
    }
    Serial.println();
}

void CycloStore::printPrimitives() const{
    Serial.println("Primitives:");
    for(uint8_t i = 0; i < _primitive_cyc_act_end; i++){
    switch(_cyclo_program[i].primitive){
        case PrimitiveCycloAction_t::FORWARD:
            Serial.print("FORWARD");
            break;
        case PrimitiveCycloAction_t::LEFT:
            Serial.print("LEFT");
            break;
        case PrimitiveCycloAction_t::RIGHT:
            Serial.print("RIGHT");
            break;
        case PrimitiveCycloAction_t::STOP:
            Serial.print("STOP");
            break;
        case PrimitiveCycloAction_t::BLANK:
            Serial.print("BLANK");
            break;
        };
        // Serial.print(toInt(_cyclo_program[i]));
        Serial.print(' ');
    }
    Serial.println();
}

void CycloStore::load_cyclograms(){
    _cyclograms[toInt(SmartCycloAction_t::STOP    )] = STOP    ;
    _cyclograms[toInt(SmartCycloAction_t::IDLE    )] = IDLE    ;
    _cyclograms[toInt(SmartCycloAction_t::FWD     )] = FWD     ;
    _cyclograms[toInt(SmartCycloAction_t::FWD_HALF)] = FWD_HALF;
    _cyclograms[toInt(SmartCycloAction_t::SS90EL  )] = SS90EL  ;
    _cyclograms[toInt(SmartCycloAction_t::SS90ER  )] = SS90ER  ;
    _cyclograms[toInt(SmartCycloAction_t::SD45SL  )] = SD45SL  ;
    _cyclograms[toInt(SmartCycloAction_t::SD45SR  )] = SD45SR  ;
    _cyclograms[toInt(SmartCycloAction_t::DS45SL  )] = DS45SL  ;
    _cyclograms[toInt(SmartCycloAction_t::DS45SR  )] = DS45SR  ;
    _cyclograms[toInt(SmartCycloAction_t::SD135SL )] = SD135SL ;
    _cyclograms[toInt(SmartCycloAction_t::SD135SR )] = SD135SR ;
    _cyclograms[toInt(SmartCycloAction_t::SS180L  )] = SS180L  ;
    _cyclograms[toInt(SmartCycloAction_t::SS180R  )] = SS180R  ;
    _cyclograms[toInt(SmartCycloAction_t::IP180   )] = IP180   ;
    _cyclograms[toInt(SmartCycloAction_t::IP90L   )] = IP90L   ;
    _cyclograms[toInt(SmartCycloAction_t::IP90R   )] = IP90R   ;
}
