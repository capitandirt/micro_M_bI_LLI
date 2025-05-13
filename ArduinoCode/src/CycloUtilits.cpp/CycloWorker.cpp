#include "CycloUtilits/CycloWorker.h"

void CycloWorker::init(){
    _cur_smart_submis = cycloStore->popFrontSmartSubmission();
    _reset_reg();
    _cyclo_context.reload();
}

void CycloWorker::doCyclogram(){
    _cur_time = millis();
    _cyclo_context.s.time = _cur_time - _last_time;

    cycloStore->executeSmart(_cur_smart_submis, &_cyclo_context);
    
    mixer->impactVelocity(_cyclo_context.ms.theta_i0, _cyclo_context.ms.v_f0);
}

bool CycloWorker::nowIsClusterDot() const{
    return _cur_smart_submis.smart == SmartCycloAction_t::CLUSTER_DOT;
}

bool CycloWorker::isCompleteCyclo() const{
    return _cyclo_context.ms.isComplete;
}

void CycloWorker::tryComplete(){
    if(_cyclo_context.ms.isComplete){
        _last_time = _cur_time; 

        _cur_smart_submis = cycloStore->popFrontSmartSubmission();

        if(_prev_smart_submis.smart != _cur_smart_submis.smart){
            _reset_reg();
        }
        _prev_smart_submis.smart = _cur_smart_submis.smart;
        
        _cyclo_context.reload();
    }
};