/* =====================================================================
   imu.cpp   MPU6050 姿态实现
   ---------------------------------------------------------------------
   用重力方向算俯仰和横滚：静态时准，船体晃动或振动大时会抖。
   偏航角这里不给，因为 MPU6050 没有磁力计，积分会漂。
   ===================================================================== */

#include "imu.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

static Adafruit_MPU6050 mpu;
static ImuData d;
static unsigned long s_lastMs = 0;

void imuBegin() {
  d.ready = false;

  Wire.begin(TOF_SDA_PIN, TOF_SCL_PIN);   // 与激光、OLED 共用总线，重复调用无副作用
  Wire.setClock(400000);

  if (!mpu.begin(IMU_ADDR, &Wire)) {
    Serial.printf("MPU6050 初始化失败：检查 VCC=3.3V、GND、SDA=GPIO%d、SCL=GPIO%d、AD0 是否接低。\n",
                  TOF_SDA_PIN, TOF_SCL_PIN);
    return;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  d.ready = true;
  Serial.printf("MPU6050 已就绪：地址 0x%02X，SDA=GPIO%d  SCL=GPIO%d\n",
                IMU_ADDR, TOF_SDA_PIN, TOF_SCL_PIN);
}

void imuUpdate() {
  if (!d.ready) return;
  if (millis() - s_lastMs < IMU_READ_MS) return;
  s_lastMs = millis();

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  d.accX = a.acceleration.x / 9.80665f;      // m/s^2 -> g
  d.accY = a.acceleration.y / 9.80665f;
  d.accZ = a.acceleration.z / 9.80665f;

  d.gyroX = g.gyro.x * 180.0f / (float)M_PI;
  d.gyroY = g.gyro.y * 180.0f / (float)M_PI;
  d.gyroZ = g.gyro.z * 180.0f / (float)M_PI;

  d.temperature = t.temperature;

  // 由重力分量算姿态角
  d.roll  = atan2f(d.accY, d.accZ) * 180.0f / (float)M_PI;
  d.pitch = atan2f(-d.accX, sqrtf(d.accY * d.accY + d.accZ * d.accZ)) * 180.0f / (float)M_PI;
}

const ImuData& imuGet() { return d; }

void imuPrintReport() {
  if (!d.ready) {
    Serial.println("姿态     : 模块未连接");
    return;
  }
  Serial.printf("姿态     : 俯仰 %.1f 度  横滚 %.1f 度  温度 %.1f C\n",
                d.pitch, d.roll, d.temperature);
}
