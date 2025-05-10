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
    
    Direction loadExplorer(Direction robot_dir);
    void primitivesToExplorers(Direction robot_dir);
    void primitivesToFasts();
    
    void reload();
    void needStartCellAligning();
    void needGetOutImpasse();
    void needStop();
private:
    enum class SmartState
    {
        FORWARD,
        RIGHT,
        LEFT,
        BACK,
        DIAG_NE,
        DIAG_NW,
        DIAG_SE,
        DIAG_SW
    };

    const PrimitiveCycloAction_t calc_primitive_cyclo_action(const uint8_t ind);
    void dirs_to_primitives();
    void start_explorer_process(Direction robot_dir);

    bool TO_IDLE();
    bool TO_STOP();
    bool TO_FWD_X();
    bool TO_SS90E();
    bool TO_SS90S();
    bool TO_SD45S_DS45S();
    bool TO_SD135S_DS45S();

    void TO_DD90X(PrimitiveCycloAction_t* TURN_TO_CHANGE); // функция для обработки X функций DD90S, она меняет TURN в зависимости от количества DD90S
    void TO_DIA_X(PrimitiveCycloAction_t* TURN_TO_CHANGE); // функция для обработки X функций DIA, она меняет TURN в зависимости от количества DIA
    int convertToSmart(); //экспериментальное
};

#endif