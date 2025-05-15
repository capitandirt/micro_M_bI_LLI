#ifndef _FUNCTIONAL_SELECTOR_H_
#define _FUNCTIONAL_SELECTOR_H_

#include "Arduino.h"
#include "Config.h"

enum class SelectorStatus : uint8_t{
    NONE, 

    EXPLORER,
    FAST,
    
    SIZE
};

class FunctionalSelector
{
public:
    FunctionalSelector(const uint8_t PIN) : _PIN(PIN) {}

    void           init()                       noexcept;
    void           tick()                       noexcept;

    SelectorStatus getStatus()                  const noexcept;
    uint8_t        getCounter()                 const noexcept;
    void           passMillis(const uint32_t t) noexcept;

private:
    void           sense()                      noexcept;
    void           plan()                       noexcept;

private:
    static constexpr uint32_t NEED_TIME_TO_DOWN = 1000; // ms
    static constexpr uint16_t ADC_THRESHOLD = 1000; // crocodiles
    mutable SelectorStatus _status = SelectorStatus::NONE;

    uint32_t _cur_millis;
    uint32_t _timer = 0;
    uint32_t _time_in_press = 0;

    uint16_t _adc_reading = 0;
    
    bool _state = 0;
    bool _prev_state = 0;

    bool _forward_front = 0;

    bool _is_press = 0;
    bool _is_down = 0;

    uint8_t _counter = 0;

    const uint8_t _PIN;
};

#endif // !_FUNCTIONAL_SELECTOR_H_