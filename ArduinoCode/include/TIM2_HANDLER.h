#ifndef _TIM2_HANDLER_H_
#define _TIM2_HANDLER_H_

#include <Arduino.h>
#include "Config.h"

namespace TIM2{
    void INIT(){
        noInterrupts();           
        // set the mode for timer 2
        bitClear(TCCR2A, WGM20);
        bitSet(TCCR2A, WGM21);
        bitClear(TCCR2B, WGM22);
        // set divisor to 128 => timer clock = 125kHz
        bitSet(TCCR2B, CS22);
        bitClear(TCCR2B, CS21);
        bitSet(TCCR2B, CS20);
        // set the timer frequency
        OCR2A = 249;  // (16000000/128/500)-1 = 249
        // enable the timer interrupt
        bitSet(TIMSK2, OCIE2A);

        interrupts(); 
    }
}

#endif