#include "CycloWorker.h"

void CycloWorker::addAction(SmartCycloAction_t action){
    if(_CYCLO_END + 1 < CYCLO_PROG_SIZE){
        _cyclo_program[_CYCLO_END++] = action;
    }
}

void CycloWorker::doCyclogram(){
    _cur_time = millis();
    _sensors.time = _cur_time - _last_time;

    _Actions_funcs[static_cast<uint8_t>(_cyclo_program[_CYCLO_COUNTER])](&_motion_states, &_sensors);
    
    mixer->impactVelocity(_motion_states.theta_i0, _motion_states.v_f0);

    if(_motion_states.isComplete){
        _CYCLO_COUNTER = (_CYCLO_COUNTER + 1) % _CYCLO_END;
        _last_time = _cur_time; 
        _sensors.robotState->reset();
        _motion_states.isComplete = 0;

    }
}

void CycloWorker::load_Actions_funcs(){
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::STOP    )] = STOP    ;
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::IDLE    )] = IDLE    ;
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::FWD     )] = FWD     ;
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::FWD_HALF)] = FWD_HALF;
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::SS90EL  )] = SS90EL  ;
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::SS90ER  )] = SS90ER  ;
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::SD45SL  )] = SD45SL  ;
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::SD45SR  )] = SD45SR  ;
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::DS45SL  )] = DS45SL  ;
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::DS45SR  )] = DS45SR  ;
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::SD135SL )] = SD135SL ;
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::SD135SR )] = SD135SR ;
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::SS180S  )] = SS180S  ;
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::IP90L   )] = IP90L   ;
    _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::IP90R   )] = IP90R   ;
}

void CycloWorker::printCycloProgram() const{
    // Serial.print("cyclo program: ");
    for(uint8_t i = 0; i < _CYCLO_END; i++){
        // Serial.print(Str_SmartCyclogramAction[static_cast<uint8_t>(_cyclo_program[i])]);
        Serial.print(static_cast<uint8_t>(_cyclo_program[i]));
        Serial.print(' ');
    }
    Serial.println();
}