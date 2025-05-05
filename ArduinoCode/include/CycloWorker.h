#ifndef _CYCLOGRAM_H_
#define _CYCLOGRAM_H_

#include <Arduino.h>

#include "Mixer.h"
#include "Config.h"

#include "CycloUtilits/CycloStore.h"

struct CycloWorkerConnectionParams{
    Mixer* mixer; 
    Odometry* odometry;  
    CycloStore* cycloStore;
    void (*_reset_reg)();
};

class CycloWorker : public CycloWorkerConnectionParams{
public:
    CycloWorker(CycloWorkerConnectionParams* cwcp) : CycloWorkerConnectionParams(*cwcp), 
                    _sensors(
                        {       
                            .time = 0,
                            .robotState = cwcp->odometry
                        }),
                    _motion_states(
                        {   
                            .v_f0 = 0,
                            .theta_i0 = 0,
                            .isComplete = 0
                        }){}


    void init();
    void doCyclogram();

    bool isComplete() const noexcept;
    void checkIsComplete();
    // void printCycloProgram() const;
private:
    Cyclogram _cur_cyclogram = IDLE;

    uint32_t _cur_time = 0;
    uint32_t _last_time = 0;
    
    Sensors _sensors;
    MotionStates _motion_states;   
    
};

#endif // !_CYCLOGRAM_H_