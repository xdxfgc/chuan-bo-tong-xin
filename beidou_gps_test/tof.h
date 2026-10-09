/* =====================================================================
   tof.h   激光测距模块 VL53L1X（I2C，双路）
   ---------------------------------------------------------------------
   两路共用一条 I2C：
     船头一只  测"正前方到岸/障碍"     —— 顶着靠、前方防撞
     右舷一只  测"船侧到码头"（右舷）   —— 侧靠，真实船舶的靠泊方式
   带 Bow 的是船头那路；旧接口名（tofIsValid 等）保留给船头，
   这样 berth / oled / net / cmd 不用全改。
   ===================================================================== */

#ifndef CB_TOF_H
#define CB_TOF_H

#include "config.h"

void     tofBegin();            // 初始化 I2C 与传感器
void     tofUpdate();           // 周期调用：按间隔读取一次距离
void     tofPrintReport();      // 打印两行距离
void     tofSelfTest();         // 串口命令 tof：两路各读 5 次，打印状态与回波强度

bool     tofIsReady();          // 船头那只是否初始化成功
bool     tofIsValid();          // 船头本次读数是否有效
uint16_t tofDistanceMm();       // 船头距离（毫米）
String   tofText();             // 船头距离文字

/* ---- 失败原因查询（给 OLED 用） ----
   OLED 只想知道一件事：这次没读数，是"看不到东西"还是"设备/线有问题"。
     · tofOutOfRange() == true  → 太远 / 没有反射面，属于正常情况，屏幕上不必写"无效"
     · tofOutOfRange() == false → 该显示出来提醒人去查（模块没起来、传感器不出数据等） */
uint8_t  tofFailCode();         // 最近一次失败的状态码（VL53L1X 的 RangeStatus）
bool     tofOutOfRange();       // 失败原因是否属于"太远/看不到东西"

/* ---- 右舷那只（TOF_SIDE_ENABLE == 0 时全是空实现） ---- */
bool     tofSideIsReady();      // 右舷那只是否初始化成功
bool     tofSideIsValid();      // 右舷本次读数是否有效
uint16_t tofSideDistanceMm();   // 右舷距离（毫米）
String   tofSideText();         // 右舷距离文字

#endif
