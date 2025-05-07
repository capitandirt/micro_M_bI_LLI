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
    
    void loadExplorer(Direction robot_dir);
    void primitivesToExplorers(Direction robot_dir);
    void primitivesToFasts();
    void needStop();
private:
    const PrimitiveCycloAction_t calc_primitive_cyclo_action(const uint8_t ind);
    void dirs_to_primitives();
    void start_explorer_process(Direction robot_dir);

    bool TO_IDLE();
    bool TO_STOP();
    bool TO_FWD();
    bool TO_SS90E();
    bool TO_SS90S();
    bool TO_SD45S_DS45S();
    bool TO_SD135S_DS45S();
    void convertToSmart(); //экспериментальное 
    
};

#endif