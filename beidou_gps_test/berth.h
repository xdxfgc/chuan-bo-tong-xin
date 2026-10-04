/* =====================================================================
   berth.h   靠泊辅助
   ---------------------------------------------------------------------
   只用激光测距，按文档附录 B 的告警码做判断：
     0x01 接近速度偏大（>0.15 m/s，提醒级）
     0x02 接近速度过大（>0.30 m/s，严重级）
     0x03 距岸过近（<0.5 米且速度 >0.10 m/s，严重级）
   另外做靠泊完成判定、距离与速度的滤波、以及语音播报的节流。
   本模块只负责“判断”，显示由 oled / net / cmd 各自去取。
   ===================================================================== */

#ifndef CB_BERTH_H
#define CB_BERTH_H

#include "config.h"

void berthBegin();               // 初始化
void berthUpdate();              // 周期调用：采样、滤波、判断、播报

bool     berthActive();          // 是否正在靠泊监测中
bool     berthDocked();          // 是否已判定靠妥
bool     berthValid();           // 当前距离是否有效
float    berthDistanceM();       // 滤波后的距离（米）
float    berthSpeedMps();        // 接近速度（米每秒，正数=正在靠近）
uint8_t  berthAlarmCode();       // 0x00 无 / 0x01 / 0x02 / 0x03
String   berthAlarmText();       // 告警文字
String   berthDistanceText();    // 距离文字，如 "1.82 m" 或 "36 cm"
void     berthPrintReport();     // 串口打印一行

#endif
