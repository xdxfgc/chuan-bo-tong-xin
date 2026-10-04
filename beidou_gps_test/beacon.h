/* =====================================================================
   beacon.h   信标接收与搜索引导
   ---------------------------------------------------------------------
   把 LoRa、方位解算、语音播报串起来：
     收包 -> 回 ACK -> 用本船北斗位置算方向与距离 -> 按节流播报 -> 网页展示
   ===================================================================== */

#ifndef CB_BEACON_H
#define CB_BEACON_H

#include "config.h"

void beaconBegin();            // 初始化信标状态
void beaconUpdate();           // 周期调用：收包、算方位、播报、判断链路超时
void beaconPrintReport();      // 打印信标链路状态

bool        beaconLinkUp();      // 链路是否在线
bool        beaconHasTarget();   // 是否收到过信标坐标
bool        beaconTargetValid(); // 信标定位是否有效
uint32_t    beaconSeq();         // 最近一包的序号
uint8_t     beaconId();          // 发送者的编号（信标 11/12/13…），0 = 老格式没带编号
const char* beaconIdText();      // 编号文字，如 “11”；没带编号时返回 “--”
double      beaconLat();
double      beaconLon();
int         beaconRssi();
float       beaconSnr();
bool        beaconHaveDir();     // 本船已定位，能给出方向
float       beaconDistM();       // 到信标的距离（米）
float       beaconBearing();     // 方位角（度）
const char* beaconDirText();     // 方位文字，如“东北”

/* 相对船头（需要磁力计在位并已标定；否则下面几个退回绝对方位） */
bool        beaconUseRel();      // 当前是否在用「相对船头」方位
float       beaconRelBearing();  // 信标相对船头的角度，0=正前方，顺时针
const char* beaconRelDirText();  // 相对方位文字，如“左前方”
float       beaconArrowBearing();// 画箭头该用的角度（能用相对就用相对）

/* 落水告警状态（供蜂鸣器汇总，需人工确认才解除） */
bool beaconAlarmActive();        // 落水告警是否处于活动状态
bool beaconEverLinked();         // 上电以来是否收到过信标包
void beaconAcknowledge();        // 人工确认，解除落水告警

#endif
