/* =====================================================================
   anchor.h   走锚监测（锚泊位移监测）
   ---------------------------------------------------------------------
   设定基准位置后，用北斗的定位做长时间平滑，再算位移与漂移趋势：
     位移超限并持续 60 秒          → 0x21 疑似走锚（提醒级）
     位移持续增大且速率没放缓      → 0x22 走锚（严重级）
   停靠在码头时，另用激光测与岸壁的距离变化做辅助（毫米级，更灵敏）。
   ===================================================================== */

#ifndef CB_ANCHOR_H
#define CB_ANCHOR_H

#include "config.h"

void anchorBegin();
void anchorUpdate();            // 周期调用：采样、平滑、判趋势、播报

void anchorSetReference();      // 把当前位置设为基准（网页按钮 / moor set）
void anchorClearReference();    // 清除基准，停止监测

bool    anchorActive();         // 是否在监测（已设基准）
double  anchorRefLat();
double  anchorRefLon();
float   anchorDriftM();         // 平滑后的位移（米）
float   anchorDriftDir();       // 漂移方向（度，0=正北）
float   anchorDriftSpeed();     // 漂移速率（米每秒）
uint8_t anchorAlarmCode();      // 0x00 / 0x21 / 0x22
String  anchorAlarmText();
String  anchorStateText();      // 供网页显示
String  anchorDriftHistory();   // 逗号分隔的历史位移，供网页画趋势曲线
void    anchorPrintReport();    // 串口打印一行

#endif
