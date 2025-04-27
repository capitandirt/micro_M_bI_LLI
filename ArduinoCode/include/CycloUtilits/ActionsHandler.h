#ifndef _ACTIONS_HANDLER_H_
#define _ACTIONS_HANDLER_H_

#include "CycloTypes.h"
#include "Maze.h"
#include "CycloStore.h"

struct ActionsHandlerConnectionParams{
    Maze* _maze;
    Odometry* _odometry;
    CycloStore* _cycloStore;
};

class ActionsHandler : ActionsHandlerConnectionParams{
public:
    ActionsHandler(ActionsHandlerConnectionParams* ahcp) : ActionsHandlerConnectionParams(*ahcp){}
    
    void primitivesToExplorers();
    void primitivesToFasts();

private:
    const PrimitiveCycloAction_t calc_primitive_cyclo_action(const uint8_t ind);
    void dirs_to_primitives();
    void start_primitive_process();

    bool TO_IDLE();
    bool TO_STOP();
    bool TO_FWD();
    bool TO_SS90E();
};

#endif