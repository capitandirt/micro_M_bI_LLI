#include "CycloWorker.h"

void CycloWorker::init(){
    _cur_smart = cycloStore->popFrontSmart();
}

void CycloWorker::doCyclogram(){
    _cur_time = millis();
    _sensors.time = _cur_time - _last_time;

    cycloStore->cyclogramFrom(_cur_smart)(&_motion_states, &_sensors);
    
    mixer->impactVelocity(_motion_states.theta_i0, _motion_states.v_f0);
}

bool CycloWorker::nowIsClusterDot() const{
    return _cur_smart == SmartCycloAction_t::CLUSTER_DOT;
}

bool CycloWorker::isCompleteCyclo() const{
    return _motion_states.isComplete;
}

void CycloWorker::checkIsComplete(){
    if(_motion_states.isComplete){
        _last_time = _cur_time; 

        _cur_smart = cycloStore->popFrontSmart();
        _sensors.robotState->updateRelative();

        _reset_reg();
        _motion_states.isComplete = 0;
    }
};