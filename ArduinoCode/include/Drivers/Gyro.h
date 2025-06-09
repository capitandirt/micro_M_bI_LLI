#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
// #include <I2Cdev.h>
// #include <MPU6050_6Axis_MotionApps20.h>
#include "Wire.h"

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
    float yaw, yaw0, yaw_offset;

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
        yaw = _pac.val / RAD_TO_DEG;

        static float yaw_old = yaw;
        if(yaw - yaw_old >= PI){ 
            yaw_offset -= 2*PI;
        }
        if(yaw - yaw_old <= -PI) {   
            yaw_offset += 2*PI;
        }
        yaw_old = yaw;

    }
    float setYaw0()
    {
        tick();
        yaw0 = yaw;
    }
    float getYawAngle()
    {
        _new_yaw = 0;

        return yaw - yaw0 + yaw_offset;
    }

    bool isNewYaw(){
        return _new_yaw;
    }

    void printYaw()
    {
        Serial.print((yaw - yaw0 + yaw_offset) * 180 / M_PI); // вокруг оси Z
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