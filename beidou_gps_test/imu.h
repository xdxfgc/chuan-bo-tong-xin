/* =====================================================================
   imu.h   MPU6050 六轴姿态（I2C）
   ---------------------------------------------------------------------
   与激光 VL53L1X(0x29)、OLED SSD1306(0x3C) 共用同一条 I2C 总线，
   MPU6050 地址 0x68，三者不冲突。
   ===================================================================== */

#ifndef CB_IMU_H
#define CB_IMU_H

#include "config.h"

struct ImuData {
  bool  ready;                 // 模块是否初始化成功
  float accX, accY, accZ;      // 加速度（g）
  float gyroX, gyroY, gyroZ;   // 角速度（度每秒）
  float pitch;                 // 俯仰角（度，抬头为正）
  float roll;                  // 横滚角（度，右倾为正）
  float temperature;           // 温度（摄氏度）
};

void imuBegin();               // 初始化（用与激光/OLED 相同的 I2C 总线）
void imuUpdate();              // 周期调用：按间隔读取一次
void imuPrintReport();         // 打印姿态到串口监视器
const ImuData& imuGet();       // 当前姿态数据

#endif
