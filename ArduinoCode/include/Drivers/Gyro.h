#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 mpu;


uint8_t fifoBuffer[45];         // буфер



class Gyro
{
    private:
    float ypr[3];
    float ypr0[3];
    public:
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

            // расчёты
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //mpu.dmpGetAccel(acc);
            // выводим результат


            // mpu.dmpGetEuler(euler, &q);
            // Serial.print("euler\t");
            // Serial.print(euler[0] * 180 / M_PI);
            // Serial.print("\t");
            // Serial.print(euler[1] * 180 / M_PI);
            // Serial.print("\t");
            // Serial.println(euler[2] * 180 / M_PI);
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
        return ypr[2] - ypr[0];
    }

    void printYPR()
    {
        Serial.print(ypr[0] * 180 / M_PI); // вокруг оси Z
        Serial.print(', ');
        Serial.print(ypr[1] * 180 / M_PI); // вокруг оси Y
        Serial.print(', ');
        Serial.println(ypr[2] * 180 / M_PI); // вокруг оси X
    }
};



#endif // GYRO_H