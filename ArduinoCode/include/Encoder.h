#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <Arduino.h>
#include "Config.h"

struct EncoderConnectionParams
{
    uint8_t CLK_A_PIN;
    uint8_t B_PIN;
    int8_t ENC_DIR;
    void (*ISR)();
};

class Encoder : public EncoderConnectionParams{
private:
    int8_t ett[4][4];
    
    volatile int counter;
    volatile int enc_old = 0;

    float phi = 0;
    float dphi = 0;

public:
    Encoder(EncoderConnectionParams *ecp) : EncoderConnectionParams(*ecp){}

    void init();
    void tick();
    
    void isr_callback();

    float GetPhi() const;
    float GetDPhi() const ;
};

#endif // !_ENCODER_H_