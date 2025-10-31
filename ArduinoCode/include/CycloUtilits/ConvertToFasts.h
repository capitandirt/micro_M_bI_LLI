#ifndef _CONVERT_TO_FASTS_H_
#define _CONVERT_TO_FASTS_H_

#include "CycloStore.h"

class ConverterToFasts
{
    private:
        enum class RobotState_t : uint8_t
        {
            LEFT45,
            ORT,
            TURN45,

            STOP,
            NAS // not a state
        };

        RobotState_t _entryHandler();
        bool _EH_SS90S_handler(PrimitiveCycloAction_t* nextPrim);
        bool _EH_SD45S_handler(PrimitiveCycloAction_t* nextPrim);
        bool _EH_SD135S_handler(PrimitiveCycloAction_t* nextPrim);
        bool _EH_SS180S_handler(PrimitiveCycloAction_t* nextPrim);
        void _EH_FWD_X_handler();

        RobotState_t _repeatActionHandler(RobotState_t startState);
        bool _RH_DD90S_handler();
        bool _RH_DIAG_X_handler();

        void _exitHandler(RobotState_t startState);
        bool _EXH_DS45S_handler(PrimitiveCycloAction_t* nextPrim);
        bool _EXH_DS135S_handler(PrimitiveCycloAction_t* nextPrim);


    public:
        ConverterToFasts(CycloStore* cycloStore) : _cycloStore(cycloStore){}
        CycloStore* _cycloStore; 
        void convert();
};

#endif // !_CONVERT_TO_FASTS_H_