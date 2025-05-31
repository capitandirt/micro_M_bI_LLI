#ifndef _FUNCTIONAL_SELECTOR_H_
#define _FUNCTIONAL_SELECTOR_H_

#include "Arduino.h"
#include "Config.h"
#include "Led.h"
#include "SlideCatcher.h"

enum class ProgramStatus : uint8_t{
    NONE = 0, 

    NEED_START_COMMAND,
    PRE_ENTRY_START,
    START_EXPLORER,
    EXPLORER,
    PRE_ENTRY_FINISH,
    START_EXPLORER_AFTER_FINISH,
    GO_TO_START,
    PRE_ENTRY_FAST,
    FAST,
    
    SIZE
};

struct StatusSelectorConnectionParams{
    const uint8_t _INPUT_PIN;
    Led* _indicator;
    SlideCatcher* _slideCatcher;
};

class StatusSelector : public StatusSelectorConnectionParams
{
public:
    StatusSelector(StatusSelectorConnectionParams* sscp):
                        StatusSelectorConnectionParams(*sscp){}

    void           init()                       noexcept;
    void           tick()                       noexcept;

    ProgramStatus  getStatus()                  const noexcept;
    void           setStatus(ProgramStatus s)   noexcept;
    void           nextStatus()                 noexcept;
    void           setNoneStatus()              noexcept;

    void           passMillis(const uint32_t t) noexcept;

private:
    void           sense()                      noexcept;
    void           plan()                       noexcept;

private:
    static constexpr uint32_t NEED_TIME_TO_DOWN = 1000; // ms
    static constexpr uint32_t TIME_IN_BLINK = Led::TIME_FULL_SATURATION;

    static constexpr uint16_t ADC_THRESHOLD = 1000; // crocodiles

    uint32_t _cur_millis = 0;
    uint32_t _timer = 0;

    uint16_t _adc_reading = 0;
    bool _prev_but_state = 0;
    uint8_t _counter = 0;
};

#endif // !_FUNCTIONAL_SELECTOR_H_