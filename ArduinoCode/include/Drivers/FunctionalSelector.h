#ifndef _FUNCTIONAL_SELECTOR_H_
#define _FUNCTIONAL_SELECTOR_H_

#include "Arduino.h"
#include "Config.h"
#include "Led.h"

enum class ProgramStatus : uint8_t{
    NONE, 

    PRE_ENTRY_START,
    START_EXPLORER,
    EXPLORER,
    PRE_ENTRY_FINISH,
    START_EXPLORER_AFTER_FINISH,
    GO_TO_START,
    FAST,
    
    SIZE
};

class FunctionalSelector
{
public:
    FunctionalSelector(const uint8_t PIN, Led* indicator):
                        _INPUT_PIN(PIN),
                        _indicator(indicator){}

    void           init()                       noexcept;
    void           tick()                       noexcept;

    ProgramStatus  getStatus()                  const noexcept;
    void           nextStatus()                  noexcept;

    void           passMillis(const uint32_t t) noexcept;

private:
    void           sense()                      noexcept;
    void           plan()                       noexcept;

private:
    static constexpr uint32_t NEED_TIME_TO_DOWN = 1000; // ms
    static constexpr uint16_t ADC_THRESHOLD = 1000; // crocodiles

    Led* _indicator;

    uint32_t _cur_millis = 0;
    uint32_t _timer = 0;
    uint32_t _time_in_press = 0;

    uint16_t _adc_reading = 0;
    
    bool _state = 0;
    bool _prev_state = 0;

    bool _forward_front = 0;

    bool _is_press = 0;
    bool _is_down = 0;

    uint8_t _counter = 0;

    const uint8_t _INPUT_PIN;
};

#endif // !_FUNCTIONAL_SELECTOR_H_