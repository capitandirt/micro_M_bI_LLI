#ifndef _TIM2_HANDLER_H_
#define _TIM2_HANDLER_H_

#include <Arduino.h>
#include "Config.h"

namespace TIM2{
    void INIT(){
        noInterrupts();           
        TCCR2A = 0;               
        TCCR2B = 0;

        const int MAX_COUNTER = 256;
        const int PRESCALER = 1024;
        const int COUNTER = Ts_us / (F_CPU / PRESCALER / 256 ) - 1;

        OCR2A = COUNTER;            
        TCCR2A |= (1 << WGM21);
        TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
        
        TIMSK2 |= (1 << OCIE2A);

        interrupts(); 
    }
}

#endif