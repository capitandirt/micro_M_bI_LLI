#ifndef GYRO_BASE_H
#define GYRO_BASE_H

#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include "Kalman.h"
#include "Config.h"

class GyroBase
{
private:
    MPU6050 mpu;

    float gyro_x_angle = 0;
    float gyro_y_angle = 0;
    float gyro_z_angle = 0;

    float kalman_x_angle = 0;
    float kalman_y_angle = 0;
    float kalman_z_angle = 0;

    Kalman kalman_x;
    Kalman kalman_y;
    Kalman kalman_z;

public:
    float correct(float angle)
    {
        if(angle < -PI) angle += 2*PI;
        if(angle > PI) angle -= 2*PI;
        return angle;
    }
    void init() 
    {
        //Serial.println("GyroBase init start");

        Wire.begin();
        Wire.setClock(400000);
        mpu.initialize();
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

        kalman_x.setAngle(0);
        kalman_y.setAngle(0);
        kalman_z.setAngle(0);

        mpu.setXAccelOffset(0);
        mpu.setYAccelOffset(0);
        mpu.setZAccelOffset(0);
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);

        calibrate();
    }

    void calibrate(){
        constexpr byte BUFFER_SIZE = 100;
        long offsets[6];
        long offsetsOld[6];
        int16_t mpuGet[6];
        // используем стандартную точность
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        // обнуляем оффсеты
        mpu.setXAccelOffset(0);
        mpu.setYAccelOffset(0);
        mpu.setZAccelOffset(0);
        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
        delay(10);

        for (byte n = 0; n < 10; n++) {     // 10 итераций калибровки
            for (byte j = 0; j < 6; j++) {    // обнуляем калибровочный массив
            offsets[j] = 0;
            }
            for (byte i = 0; i < 100 + BUFFER_SIZE; i++) { // делаем BUFFER_SIZE измерений для усреднения
            mpu.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]);
            if (i >= 99) {                         // пропускаем первые 99 измерений
                for (byte j = 0; j < 6; j++) {
                offsets[j] += (long)mpuGet[j];   // записываем в калибровочный массив
                }
            }
            }
            for (byte i = 0; i < 6; i++) {
            offsets[i] = offsetsOld[i] - ((long)offsets[i] / BUFFER_SIZE); // учитываем предыдущую калибровку
            if (i == 2) offsets[i] += 16384;                               // если ось Z, калибруем в 16384
            offsetsOld[i] = offsets[i];
            }
            // ставим новые оффсеты
            mpu.setXAccelOffset(offsets[0] / 8);
            mpu.setYAccelOffset(offsets[1] / 8);
            mpu.setZAccelOffset(offsets[2] / 8);
            mpu.setXGyroOffset(offsets[3] / 4);
            mpu.setYGyroOffset(offsets[4] / 4);
            mpu.setZGyroOffset(offsets[5] / 4);
            delay(2);
        }
    }

    void tick() 
    {
        const float acc_x = mpu.getAccelerationX();
        const float acc_y = mpu.getAccelerationY();
        const float acc_z = mpu.getAccelerationZ();

        const float gyro_x = mpu.getRotationX();
        const float gyro_y = mpu.getRotationY();
        const float gyro_z = mpu.getRotationZ();

        const float acc_x_angle = (atan2(acc_y, acc_z) + PI) * RAD_TO_DEG;
        const float acc_y_angle = (atan2(acc_x, acc_z) + PI) * RAD_TO_DEG;
        const float acc_z_angle = (atan2(acc_y, acc_x) + PI) * RAD_TO_DEG;

        const float gyro_x_rate = gyro_x / 131.f;
        const float gyro_y_rate = -gyro_y / 131.f;
        const float gyro_z_rate = gyro_z / 131.f;

        gyro_x_angle += gyro_x_rate * Ts_s;
        gyro_y_angle += gyro_y_rate * Ts_s;
        gyro_z_angle += gyro_z_rate * Ts_s;

        kalman_x_angle = kalman_x.getAngle(acc_x_angle, gyro_x_rate, Ts_s);
        kalman_y_angle = kalman_y.getAngle(acc_y_angle, gyro_y_rate, Ts_s);
        kalman_z_angle = (kalman_z.getAngle(acc_z_angle, gyro_z_rate, Ts_s) - kalman_z_angle) * 0.30;

        // Serial.print(kalman_x_angle);
        // Serial.print('\t'); 
        // Serial.print(kalman_y_angle);
        // Serial.print('\t');
        // Serial.println(kalman_z_angle);
    }
};



#endif // !GYRO_BASE_H