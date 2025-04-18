#ifndef _CYCLOGRAM_H_
#define _CYCLOGRAM_H_

#include <Arduino.h>

#include "Mixer.h"
#include "Config.h"

#include "CycloUtilits/CycloActions.h"

struct CycloWorkerConnectionParams{
    Mixer* mixer; 
    Odometry* odometry;  
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

    void addAction(SmartCycloAction_t action);
    void doCyclogram();

    void printCycloProgram();
    
    void loadActionsFuncs();
    
private:
    static constexpr uint8_t CYCLO_PROG_SIZE = 64;

    SmartCycloAction_t _cyclo_program[CYCLO_PROG_SIZE];  
    
    CycloAction _Actions_funcs[static_cast<uint8_t>(SmartCycloAction_t::CYCLO_ACTION_SIZE)];
    
    uint8_t _CYCLO_COUNTER = 0;
    uint8_t _CYCLO_END = 0;
      
    uint32_t _cur_time = 0;
    uint32_t _last_time = 0;
    
    Sensors _sensors;
    MotionStates _motion_states;   
    
};

#endif // !_CYCLOGRAM_H_