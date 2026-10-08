/* =====================================================================
   motor.h   TB6612 电机驱动（单路，A 通道，STBY 常开）
   ---------------------------------------------------------------------
   接线见 config.h 的 MOTOR_* 段。单片机只管三根线：
     AIN1 / AIN2 决定方向，PWMA 决定转速（LEDC，5kHz）。

   本模块做了三件保护，都是照船模的实际需要来的：
     1) 软启动 / 软停：油门每秒最多变化 MOTOR_RAM  —— 突然全速会打滑甚至翻船
     2) 换向保护：要倒车时先把转速降到 0、停稳 300ms 再给反向油门
     3) 死区：油门小于 6% 直接当停车，不然电机会一直嗡嗡响

   上电默认停止，只有明确下命令才动。                                   */

#ifndef CB_MOTOR_H
#define CB_MOTOR_H

#include "config.h"

void  motorBegin();                 // 初始化引脚与 PWM（上电即停止）
void  motorUpdate();                // 周期调用：斜坡、换向保护、输出到 H 桥

void  motorSetThrottle(float t);    // 目标油门：-1.0（全速倒车）~ +1.0（全速前进）
void  motorStop();                  // 滑行停（两路都低，电机自由转）
void  motorBrake();                 // 刹车（两路都高，电机被短接，停得快）
void  motorEmergencyStop();         // 急停：目标清零 + 立即刹车

bool  motorReady();                 // 驱动是否可用
float motorTarget();                // 目标油门
float motorOutput();                // 实际输出油门（斜坡之后）
const char* motorStateText();       // “滑行 / 前进 50% / 倒车 30% / 刹车”
String motorCmd(const String& arg); // 串口命令 motor 的处理
bool  motorLocked();                // 是否处于靠泊严重告警联锁

#endif
