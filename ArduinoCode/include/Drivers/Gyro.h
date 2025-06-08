#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

class Gyro
{
    private:
    MPU6050 mpu;
    uint8_t fifoBuffer[45];         // буфер
    float ypr[3];
    float ypr0[3];
    float ypr_offset[3];
    public:
    float correct(float angle)
    {
        if(angle < -PI) angle += 2*PI;
        if(angle > PI) angle -= 2*PI;
        return angle;
    }
    void init() 
    {
        Wire.begin();
        Wire.setClock(400000);
        //Wire.setClock(1000000UL);   // разгоняем шину на максимум

        // инициализация DMP
        mpu.initialize();
        mpu.dmpInitialize();
        mpu.setDMPEnabled(true);
    }

    void tick() 
    {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            // переменные для расчёта (ypr можно вынести в глобал)
            Quaternion q;
            VectorFloat gravity;
            static float ypr_old[3] = {0, 0, 0};

            // расчёты
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //mpu.dmpGetAccel(acc);
            // выводим результат

            if(ypr[0] - ypr_old[0] > PI) ypr_offset[0] -= 2*PI;
            if(ypr[0] - ypr_old[0] < -PI) ypr_offset[0] += 2*PI;
            
            ypr_old[0] = ypr[0];
            ypr_old[1] = ypr[1];
            ypr_old[2] = ypr[2];
        }
    }
    float setYPR0()
    {
        tick();
        ypr0[0] = ypr[0];
        ypr0[1] = ypr[1];
        ypr0[2] = ypr[2];
    }
    float getYawAngle()
    {
        return ypr[0] - ypr0[0] + ypr_offset[0];
    }

    void printYPR()
    {
        Serial.print((ypr[0] - ypr0[0]) * 180 / M_PI); // вокруг оси Z 
        Serial.print(', ');
        Serial.print((ypr[1] - ypr0[1]) * 180 / M_PI); // вокруг оси Y
        Serial.print(', ');
        Serial.println((ypr[2] - ypr0[2]) * 180 / M_PI); // вокруг оси X
    }

    void setYawOffset(float offset)
    {
        ypr_offset[0] = offset;
    }
    void modifyYawOffset(float offset)
    {
        ypr_offset[0] += offset;
    }
};



#endif // GYRO_H