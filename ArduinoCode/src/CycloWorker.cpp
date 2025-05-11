#include "CycloWorker.h"

void CycloWorker::init(){
    _cur_smart_submis = cycloStore->popFrontSmartSubmission();
}

void CycloWorker::doCyclogram(){
    _cur_time = millis();
    _cyclo_context.s.time = _cur_time - _last_time;

    cycloStore->executeSmart(_cur_smart_submis, _cyclo_context);
    
    mixer->impactVelocity(_cyclo_context.ms.theta_i0, _cyclo_context.ms.v_f0);
}

bool CycloWorker::nowIsClusterDot() const{
    return _cur_smart_submis.smart == SmartCycloAction_t::CLUSTER_DOT;
}

bool CycloWorker::isCompleteCyclo() const{
    return _cyclo_context.ms.isComplete;
}

void CycloWorker::checkIsComplete(){
    if(_cyclo_context.ms.isComplete){
        _last_time = _cur_time; 

        _cur_smart_submis = cycloStore->popFrontSmartSubmission();
        _cyclo_context.s.robotState->updateRelative();

        _reset_reg();
        _cyclo_context.ms.isComplete = 0;
    }
};