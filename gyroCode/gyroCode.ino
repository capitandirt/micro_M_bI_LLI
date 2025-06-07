#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

uint8_t fifoBuffer[45];         // буфер

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  //Wire.setClock(1000000UL);   // разгоняем шину на максимум

  // инициализация DMP
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
}

void loop() {
  static uint32_t tmr;
  if (millis() - tmr >= 11) {  // таймер на 11 мс (на всякий случай)
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // переменные для расчёта (ypr можно вынести в глобал)
      Quaternion q;
      VectorFloat gravity;
      float ypr[3];
      float euler[3];

      // расчёты
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // выводим результат
      Serial.print(ypr[0] * 180 / M_PI); // вокруг оси Z
      Serial.print(',');
      Serial.print(ypr[1] * 180 / M_PI); // вокруг оси Y
      Serial.print(',');
      Serial.print(ypr[2] * 180 / M_PI); // вокруг оси X
      Serial.println();
      // mpu.dmpGetEuler(euler, &q);
      // Serial.print("euler\t");
      // Serial.print(euler[0] * 180 / M_PI);
      // Serial.print("\t");
      // Serial.print(euler[1] * 180 / M_PI);
      // Serial.print("\t");
      // Serial.println(euler[2] * 180 / M_PI);

      tmr = millis();  // сброс таймера
    }
  }
}
