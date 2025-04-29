#include "Encoder.h"

void Encoder::init()
{
    pinMode(CLK_A_PIN, INPUT);
    pinMode(B_PIN, INPUT);

    noInterrupts();
    attachInterrupt(
        digitalPinToInterrupt(CLK_A_PIN),
        ISR,
        CHANGE
    );

    _ett[0b00][0b01] = ENC_DIR;
    _ett[0b01][0b11] = ENC_DIR;
    _ett[0b11][0b10] = ENC_DIR;
    _ett[0b10][0b00] = ENC_DIR;
    
    _ett[0b00][0b10] = -ENC_DIR;
    _ett[0b10][0b11] = -ENC_DIR;
    _ett[0b11][0b01] = -ENC_DIR;
    _ett[0b01][0b00] = -ENC_DIR;
    
    interrupts();
}

void Encoder::tick()
{
    noInterrupts();
    const int counter_inc = _counter;
    _counter = 0;
    interrupts();

    _dphi = counter_inc * TICK2RAD;
    _phi += _dphi;
}

void Encoder::isrCallback()
{
    const uint8_t B = digitalRead(B_PIN);
    const uint8_t CLK_A = digitalRead(CLK_A_PIN);
    const uint8_t A = CLK_A ^ B;
    const uint8_t enc = (A << 1) | B;

    _counter += _ett[_enc_old][enc];
    _enc_old = enc;
}

float Encoder::getPhi() const{
    return _phi;
}

float Encoder::getDPhi() const{
    return _dphi;
}