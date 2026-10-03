/* =====================================================================
   tof.h   激光测距模块 VL53L1X（I2C）
   ===================================================================== */

#ifndef CB_TOF_H
#define CB_TOF_H

#include "config.h"

void     tofBegin();            // 初始化 I2C 与传感器
void     tofUpdate();           // 周期调用：按间隔读取一次距离
void     tofPrintReport();      // 打印一行距离

bool     tofIsReady();          // 模块是否初始化成功
bool     tofIsValid();          // 本次读数是否有效
uint16_t tofDistanceMm();       // 距离（毫米）
String   tofText();             // 供网页显示的文字

#endif
