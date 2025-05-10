#ifndef _CYCLOGRAM_H_
#define _CYCLOGRAM_H_

#include <Arduino.h>

#include "Mixer.h"
#include "Config.h"
#include "OptocouplerSensors.h"

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
    CycloWorker(CycloWorkerConnectionParams* cwcp) : CycloWorkerConnectionParams(*cwcp), 
                    _sensors(
                        {       
                            .time = 0,
                            .robotState = cwcp->odometry,
                            .optocoupler = cwcp->optocoupler,
                        }),
                    _motion_states(
                        {   
                            .v_f0 = 0,
                            .theta_i0 = 0,
                            .isComplete = false
                        }){}


    void init();
    void doCyclogram();

    bool nowIsStop() const noexcept;
    bool isCompleteCyclo() const noexcept;

    void checkIsComplete();

private:
    SmartCycloAction_t _cur_smart = SmartCycloAction_t::IDLE;

    uint32_t _cur_time = 0;
    uint32_t _last_time = 0;
    
    Sensors _sensors;
    MotionStates _motion_states;   
};

#endif // !_CYCLOGRAM_H_