#include "CycloUtilits/CycloStore.h"

void CycloStore::addSmart(const SmartCycloAction_t smart, uint8_t x = toInt(X_t::NONE)){ 
    if(smart == SmartCycloAction_t::FWD_X || 
       smart == SmartCycloAction_t::DIAG_X)
    {
        if(_smart_cyc_act_end + 1 < CYCLO_PROG_SIZE){
            _cyclo_program[_smart_cyc_act_end++].smart = smart;
            _cyclo_program[_smart_cyc_act_end++].smart = static_cast<SmartCycloAction_t>(toX_t(x)); // if NONE, than FWD_1
        }
    }
    else
    {
        if(_smart_cyc_act_end < CYCLO_PROG_SIZE){
            _cyclo_program[_smart_cyc_act_end++].smart = smart;
        }

        for(uint8_t i = 1; i < x; i++){
            if(_smart_cyc_act_end < CYCLO_PROG_SIZE){
                _cyclo_program[_smart_cyc_act_end++].smart = smart;
            }
        }
    }
}

void CycloStore::addPrimitive(PrimitiveCycloAction_t primitive){
    if(_primitive_cyc_act_counter < CYCLO_PROG_SIZE){
        _cyclo_program[_primitive_cyc_act_end++].primitive = primitive;
    }
}

SmartSubmission CycloStore::popFrontSmartSubmission(){
    if(_smart_cyc_act_counter >= _smart_cyc_act_end) return {SmartCycloAction_t::IDLE, X_t::NONE};
    
    SmartSubmission smart_submis = {_cyclo_program[_smart_cyc_act_counter++].smart, X_t::NONE};
    
    if(smart_submis.smart == SmartCycloAction_t::FWD_X){
        smart_submis.x = toX_t(_cyclo_program[_smart_cyc_act_counter++].smart);
    }

    return smart_submis;
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
 
void CycloStore::executeSmart(SmartSubmission smart_submis, CycloContext* cyclo_context){
    _cyclograms[toInt(smart_submis.smart)](&cyclo_context->ms, &cyclo_context->s, toInt(smart_submis.x));
}

void CycloStore::printSmarts() const{
    Serial.println("Smarts:");
    for(uint8_t i = _smart_cyc_act_counter; i < _smart_cyc_act_end; i++){
        SmartCycloAction_t smart_action = _cyclo_program[i].smart;
        Serial.print(Str_SmartCyclogramAction[toInt(smart_action)]);
        
        if(smart_action == SmartCycloAction_t::FWD_X ||
           smart_action == SmartCycloAction_t::DIAG_X){
            X_t x = toX_t(_cyclo_program[++i].smart);
            Serial.print(toInt(x));
        }

        Serial.print(' ');
    }
    Serial.println();
    Serial.println();
}

void CycloStore::printPrimitives() const{
    Serial.println("Primitives:");
    for(uint8_t i = _primitive_cyc_act_counter; i < _primitive_cyc_act_end; i++){
    switch(_cyclo_program[i].primitive){
        case PrimitiveCycloAction_t::FORWARD:
            Serial.print("FORWARD");
            break;
        case PrimitiveCycloAction_t::LEFT:
            Serial.print("LEFT");
            break;
        case PrimitiveCycloAction_t::BACK:
            Serial.print("BACK");
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
        Serial.print(' ');
    }
    Serial.println();
    Serial.println();
}

void CycloStore::load_cyclograms(){
    _cyclograms[toInt(SmartCycloAction_t::STOP                )] = STOP                ;
    _cyclograms[toInt(SmartCycloAction_t::CLUSTER_DOT         )] = CLUSTER_DOT         ;
    _cyclograms[toInt(SmartCycloAction_t::IDLE                )] = IDLE                ;
    _cyclograms[toInt(SmartCycloAction_t::FWD_HALF            )] = FWD_HALF            ;
    _cyclograms[toInt(SmartCycloAction_t::FWDE                )] = FWDE                ;
    _cyclograms[toInt(SmartCycloAction_t::FWD_X               )] = FWD_X               ;
    _cyclograms[toInt(SmartCycloAction_t::DIAG_X              )] = DIAG_X              ;
    _cyclograms[toInt(SmartCycloAction_t::SS90EL              )] = SS90EL              ;
    _cyclograms[toInt(SmartCycloAction_t::SS90ER              )] = SS90ER              ;
    _cyclograms[toInt(SmartCycloAction_t::SS90SR              )] = SS90SR              ;
    _cyclograms[toInt(SmartCycloAction_t::SS90SL              )] = SS90SL              ;
    _cyclograms[toInt(SmartCycloAction_t::SD45SL              )] = SD45SL              ;
    _cyclograms[toInt(SmartCycloAction_t::SD45SR              )] = SD45SR              ;
    _cyclograms[toInt(SmartCycloAction_t::DS45SL              )] = DS45SL              ;
    _cyclograms[toInt(SmartCycloAction_t::DS45SR              )] = DS45SR              ;
    _cyclograms[toInt(SmartCycloAction_t::DD90SL              )] = DD90SL              ;
    _cyclograms[toInt(SmartCycloAction_t::DD90SR              )] = DD90SR              ;
    _cyclograms[toInt(SmartCycloAction_t::SD135SL             )] = SD135SL             ;
    _cyclograms[toInt(SmartCycloAction_t::SD135SR             )] = SD135SR             ;
    _cyclograms[toInt(SmartCycloAction_t::DS135SL             )] = DS135SL             ;
    _cyclograms[toInt(SmartCycloAction_t::DS135SR             )] = DS135SR             ;
    _cyclograms[toInt(SmartCycloAction_t::SS180SL             )] = SS180SL             ;
    _cyclograms[toInt(SmartCycloAction_t::SS180SR             )] = SS180SR             ;
    _cyclograms[toInt(SmartCycloAction_t::IP180               )] = IP180               ;
    _cyclograms[toInt(SmartCycloAction_t::IP90L               )] = IP90L               ;
    _cyclograms[toInt(SmartCycloAction_t::IP90R               )] = IP90R               ;
    _cyclograms[toInt(SmartCycloAction_t::TO_ALIGN            )] = TO_ALIGN            ;
    _cyclograms[toInt(SmartCycloAction_t::FROM_ALIGN_TO_CENTER)] = FROM_ALIGN_TO_CENTER;
}
