#ifndef _FUNCTIONAL_SELECTOR_H_
#define _FUNCTIONAL_SELECTOR_H_

#include "Arduino.h"
#include "Config.h"
#include "Led.h"
#include "OptocouplerSensors.h"

enum class ProgramStatus : uint8_t{
    NONE = 0, 

    PRE_ENTRY_START,
    START_EXPLORER,
    EXPLORER,
    PRE_ENTRY_FINISH,
    START_EXPLORER_AFTER_FINISH,
    GO_TO_START,
    FAST,
    
    SIZE
};

struct StatusSelectorConnectionParams{
   const uint8_t _INPUT_PIN;
    Led* _indicator;
    OptocouplerSensors* _optocouplers;
};

class StatusSelector : public StatusSelectorConnectionParams
{
public:
    StatusSelector(StatusSelectorConnectionParams* sscp):
                        StatusSelectorConnectionParams(*sscp){}

    void           init()                       noexcept;
    void           tick()                       noexcept;

    ProgramStatus  getStatus()                  const noexcept;
    void           nextStatus()                 noexcept;
    void           setNoneStatus()              noexcept;

    bool           isSlideFromOpto()            noexcept;

    void           passMillis(const uint32_t t) noexcept;

private:
    void           sense()                      noexcept;
    void           plan()                       noexcept;

    void           work_with_button()           noexcept;
    void           work_with_opto()             noexcept;


private:
    static constexpr uint32_t NEED_TIME_TO_DOWN = 1000; // ms
    static constexpr uint16_t ADC_THRESHOLD = 1000; // crocodiles

    uint32_t _cur_millis = 0;
    uint32_t _timer = 0;
    uint32_t _time_in_press = 0;

    /*==BUTTON HANDLER==*/
    uint16_t _adc_reading = 0;
    bool _prev_but_state = 0;
    uint8_t _but_counter = 0;

    /*==OPTOCOUPLERS HANDLER==*/
    enum OptoPeriod { ZERO, FRONT, IN_OPEN, CLOSE, SLIDE } 
        _opto_previod;     
    bool _prev_opto_state = 0;
    bool _is_slide = 0;
};

#endif // !_FUNCTIONAL_SELECTOR_H_