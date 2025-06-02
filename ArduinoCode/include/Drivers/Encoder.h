#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <Arduino.h>
#include "Config.h"

struct EncoderConnectionParams
{
    const uint8_t CLK_A_PIN;
    const uint8_t B_PIN;
    const int8_t ENC_DIR;
    void (*ISR)();
};

class Encoder : private EncoderConnectionParams{
public:
    Encoder(EncoderConnectionParams *ecp) : EncoderConnectionParams(*ecp){}

    void init();
    void tick();
    
    void isrCallback();

    float getPhi() const;
    float getDPhi() const;
    
private:
    int8_t _ett[4][4];
    
    volatile int _counter;
    volatile int _enc_old = 0;

    float _phi = 0;
    float _dphi = 0;
};

#endif // !_ENCODER_H_