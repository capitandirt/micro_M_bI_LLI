#include "CycloWorker.h"

void CycloWorker::doCyclogram(){
    _cur_time = millis();
    _sensors.time = _cur_time - _last_time;

    _cur_cyclogram(&_motion_states, &_sensors);
    
    mixer->impactVelocity(_motion_states.theta_i0, _motion_states.v_f0);
}

bool CycloWorker::isComplete() const {
    return _motion_states.isComplete;
}

void CycloWorker::checkIsComplete(){
    if(_motion_states.isComplete){
        _last_time = _cur_time; 

        _cur_cyclogram = cycloStore->cyclogramFrom();
        _sensors.robotState->resetRelative();
        _motion_states.isComplete = 0;
    }
};