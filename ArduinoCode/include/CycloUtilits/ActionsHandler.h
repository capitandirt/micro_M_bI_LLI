#ifndef _ACTIONS_HANDLER_H_
#define _ACTIONS_HANDLER_H_

#include "CycloTypes.h"
#include "Maze.h"
#include "CycloStore.h"

struct ActionsHandlerConnectionParams{
    Maze* _maze;
    CycloStore* _cycloStore;
};

class ActionsHandler : ActionsHandlerConnectionParams{
public:
    ActionsHandler(ActionsHandlerConnectionParams* ahcp) : ActionsHandlerConnectionParams(*ahcp){}
    
    void primitivesToExplorers(Direction robot_dir, Vec2 robot_coords, Vec2& changed_coords);
    void primitivesToFasts();

private:
    const PrimitiveCycloAction_t calc_primitive_cyclo_action(const uint8_t ind);
    void dirs_to_primitives();
    void start_explorer_process(Direction robot_dir, Vec2 robot_coords, Vec2& changed_coords);

    bool TO_IDLE();
    bool TO_STOP();
    bool TO_FWD();
    bool TO_SS90E();
    bool TO_SS90S();
    bool TO_FROM_DIAGS_TO_OP_DIAGS();
};

#endif