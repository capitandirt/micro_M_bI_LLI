#ifndef _ACTIONS_HANDLER_H_
#define _ACTIONS_HANDLER_H_

#include "CycloTypes.h"
#include "Maze.h"
#include "CycloStore.h"
#include "MazeObserver.h"

struct ActionsHandlerConnectionParams{
    Maze* _maze;
    CycloStore* _cycloStore;
    MazeObserver* _mazeObserver;
};

class ActionsHandler : ActionsHandlerConnectionParams{
public:
    ActionsHandler(ActionsHandlerConnectionParams* ahcp) : ActionsHandlerConnectionParams(*ahcp){}
    
    void loadExplorer(Direction robot_dir);
    void primitivesToFasts();
    
    void clear();
    void needStartCellAligning();
    void needClusterDot();
    void needEnd();
    void needFwdHalf();
    void needDelay05();
    void needIdle();

    Direction needTurn(Direction dir);
    Direction needDirection(const Direction cur,  const Direction need);
    
    void convertToSmart(); //экспериментальное
private:
    PrimitiveCycloAction_t calc_primitive_cyclo_action(const uint8_t ind);
    void dirs_to_primitives();

    bool TO_IDLE();
    bool TO_STOP();
    bool TO_FWD_X();
    bool TO_FWD_X_TEMPLATE();
    bool TO_SS90E(bool isStart);
    bool TO_SS90S();
    bool TO_SD45S_DS45S(bool isStart);
    bool TO_SD135S_DS45S(bool isStart);

    enum class RobotState_t : uint8_t
    {
        LEFT,
        FORWARD,
        RIGHT,

        STOP,
        NAS // not a state
    };
    RobotState_t toState(const PrimitiveCycloAction_t curPrim);
    PrimitiveCycloAction_t fromState(const RobotState_t rs);
    uint8_t toIntFromState(const RobotState_t rs);

    RobotState_t TO_DD90X(const RobotState_t startState); 
    RobotState_t TO_DIA_X(const RobotState_t startState); 
    
    RobotState_t entryHandler(); //возвращает состояние робота на конец входа. Stop если кластер обработан
    RobotState_t repeatActionHandler(const RobotState_t startState); 
    RobotState_t exitHandler(const RobotState_t startState);
};

#endif