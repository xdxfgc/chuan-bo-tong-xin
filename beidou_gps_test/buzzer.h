/* =====================================================================
   buzzer.h   蜂鸣器（MH-FMG 有源蜂鸣器，高电平触发）
   ---------------------------------------------------------------------
   把三个模块的告警汇总成一个等级，用不同的响铃节奏区分：
       3 = 人员落水      三短声急促循环
       2 = 靠泊严重 / 信标失联   两短声循环
       1 = 靠泊提醒 / 定位失锁   单短声，每 2 秒
       0 = 无告警        静音

   网页上有一个开关可以随时关掉蜂鸣器；关掉之后如果告警解除、或者出现更高
   等级的告警，它会自动恢复。
   ===================================================================== */

#ifndef CB_BUZZER_H
#define CB_BUZZER_H

#include "config.h"

void buzzerBegin();
void buzzerUpdate();              // 周期调用：汇总告警、按节奏驱动引脚

void buzzerSetUserOn(bool on);    // 网页开关：用户手动开/关
bool buzzerUserOn();              // 用户开关的状态
bool buzzerSounding();            // 此刻是否正在响
int  buzzerLevel();               // 当前告警等级 0~3
String buzzerLevelText();         // 等级文字，如“人员落水”
String buzzerStateText();         // 供网页显示的一句话状态

void buzzerAcknowledge();         // 确认落水告警（等同于 ack 命令）

#endif
