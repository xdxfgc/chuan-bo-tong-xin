/* =====================================================================
   button.h   板载 BOOT 键（长按复位）
   ---------------------------------------------------------------------
   文档 6.3：信标打捞上来之后擦干，长按复位键五秒退出激活状态。
   这里直接用板子上现成的 BOOT 键（GPIO9），不用另外加硬件。

   ⚠ 上电时别按着 BOOT —— 按住 BOOT 上电，芯片会进下载模式，程序不跑。
   ===================================================================== */

#ifndef CB_BUTTON_H
#define CB_BUTTON_H

#include "config.h"
#include "debug.h"

void buttonBegin();          // 配脚
void buttonUpdate();         // 周期调用：消抖、判长按

bool buttonPressed();        // 此刻是否按着
bool buttonResetEvent();     // 长按到 5 秒了没有（读一次就清，用来触发复位）
unsigned long buttonHoldMs();// 已经按住多久（毫秒），没按返回 0

#endif
