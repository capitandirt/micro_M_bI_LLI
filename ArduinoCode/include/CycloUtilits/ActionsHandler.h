#ifndef _ACTIONS_HANDLER_H_
#define _ACTIONS_HANDLER_H_

#include "CycloTypes.h"
#include "Maze.h"
#include "CycloStore.h"

struct ActionsHandlerConnectionParams{
    Maze* _maze;
    CycloStore* _cycloStore;
};

class ActionsHandler : private ActionsHandlerConnectionParams{
public:
    ActionsHandler(ActionsHandlerConnectionParams* ahcp) : ActionsHandlerConnectionParams(*ahcp){}
    
    void loadExplorer(Direction robot_dir);
    void primitivesToExplorers(Direction robot_dir);
    void primitivesToFasts();
    
    void reload();
    void needStartCellAligning();
    void needGetOutImpasse();
    void needStop();
    
    int convertToSmart(); //экспериментальное
private:

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

    enum class RobotState_t : uint8_t
    {
        LEFT,
        LEFT_FORWARD,
        FORWARD,
        RIGHT_FORWARD,
        RIGHT,

        STOP,
        NAS // not a state
    };
    inline RobotState_t toState(const PrimitiveCycloAction_t curPrim)
    {
        switch(curPrim)
        {
            case PrimitiveCycloAction_t::LEFT:
                return RobotState_t::LEFT;
            case PrimitiveCycloAction_t::RIGHT:
                return RobotState_t::RIGHT;
            case PrimitiveCycloAction_t::FORWARD:
                return RobotState_t::FORWARD;
                
            default:
                return RobotState_t::NAS;// этого не может быть
        }
    }
    inline uint8_t toIntFromState(const RobotState_t rs){
        return static_cast<uint8_t>(rs);
    }

    RobotState_t TO_DD90X(const RobotState_t startState); 
    RobotState_t TO_DIA_X(const RobotState_t startState); 
    
    RobotState_t entryHandler(); //возвращает состояние робота на конец входа. Stop если кластер обработан
    RobotState_t repeatActionHandler(const RobotState_t startState); 
    RobotState_t exitHandler(const RobotState_t startState);
};

#endif