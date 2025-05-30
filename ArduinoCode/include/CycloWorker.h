#ifndef _CYCLOGRAM_H_
#define _CYCLOGRAM_H_

#include <Arduino.h>

#include "Mixer.h"
#include "Config.h"
#include "Drivers/OptocouplerSensors.h"

#include "CycloUtilits/CycloStore.h"

struct CycloWorkerConnectionParams{
    Mixer* mixer; 
    Odometry* odometry;  
    CycloStore* cycloStore;
    OptocouplerSensors* optocoupler;
    void (*_reset_reg)();
};

class CycloWorker : public CycloWorkerConnectionParams{
public:
    CycloWorker(CycloWorkerConnectionParams* cwcp) :
        CycloWorkerConnectionParams(*cwcp), 
        _cyclo_context {
            .ms = { .v_f0 = 0,
                    .theta_i0 = 0,
                    .isComplete = false,
                    .theta_0 = 0},

            .s = {  .time = 0,
                    .odometry = cwcp->odometry,
                    .optocoupler = cwcp->optocoupler}}{}

    void reload();
    void doCyclogram();

    bool nowIsClusterDot() const noexcept;
    bool isCompleteCyclo() const noexcept;

    void tryComplete();

private:
    SmartSubmission _cur_smart_submis = {SmartCycloAction_t::CLUSTER_DOT, X_t::NONE};
    SmartSubmission _prev_smart_submis = {SmartCycloAction_t::CLUSTER_DOT, X_t::NONE};

    uint32_t _cur_time = 0;
    uint32_t _last_time = 0;
    
    CycloContext _cyclo_context;
};

#endif // !_CYCLOGRAM_H_