#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include "Wire.h"

inline float circle_mod(float angle){
    while(angle > PI) angle -= 2 * PI;
    while(angle < -PI) angle += 2 * PI;

    return angle;
}

class Gyro
{
private:
    union Packet{
        uint16_t val;
        uint8_t val_raw[sizeof(val)];
    } _pac;

    static constexpr uint8_t PACKET_SIZE = sizeof(_pac);
    void (*_recieve_event)(int) = nullptr;
    volatile bool _new_yaw = 0;

    // MPU6050 mpu
    float yaw_raw, yaw0 = 0, yaw_offset = 0;
    bool yaw0_set = false;
public:
    Gyro(void (*recieve_event)(int)) : _recieve_event(recieve_event){}

    void recieveEvent(int bytes)
    {
        if(bytes != PACKET_SIZE) return;

        _new_yaw = 1;

        for(uint8_t i = 0; i < PACKET_SIZE; i++){
            _pac.val_raw[i] = Wire.read();
        }
    }

    void init()
    {
        Wire.begin(9);
        Wire.onReceive(_recieve_event);
    }
    void tick()
    {
        yaw_raw = _pac.val / RAD_TO_DEG;
        static float yaw_raw_old = yaw_raw;

        if(yaw0_set)
        {
            if(yaw_raw - yaw_raw_old >= PI){ 
                yaw_offset -= 2*PI;
            }
            if(yaw_raw - yaw_raw_old <= -PI) {   
                yaw_offset += 2*PI;
            }
        }
        yaw_raw_old = yaw_raw;

    }
    void setYaw0()
    {
        tick();
        yaw0 = yaw_raw + yaw_offset;
        yaw0_set = true;
    }
    float getYawAngle()
    {
        _new_yaw = 0;
        // Serial.println("getYawAngle: " + String(yaw_raw) + " " + String(yaw0) + " " + String(yaw_offset) + " " + String(yaw_raw - yaw0 + yaw_offset));
        return circle_mod(yaw_raw - yaw0 + yaw_offset);
    }

    bool isNewYaw(){
        return _new_yaw;
    }

    void printYaw()
    {
        Serial.print((yaw_raw - yaw0 + yaw_offset) * 180 / M_PI); // вокруг оси Z
    }

    void setYawOffset(float offset)
    {
        yaw_offset = offset;
    }
    void modifyYawOffset(float offset)
    {
        yaw_offset += offset;
    }
};



#endif // GYRO_H