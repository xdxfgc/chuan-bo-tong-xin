/* =====================================================================
   posture.h   姿态判定（MPU6050 六轴）
   ---------------------------------------------------------------------
   三重确认里的第二条：判断"是不是水面漂浮姿态"。

   它不是主触发，是**抑制误报**用的：
     在船上被雨淋、上浪、冲洗甲板时——水感会导通，但人的姿态没变。
     真的落水了——姿态会从直立翻成躺姿，而且入水瞬间有"失重→冲击"的波形。

   接线和阈值见 config.h 里那一节。
   ===================================================================== */

#ifndef CB_POSTURE_H
#define CB_POSTURE_H

#include "config.h"
#include "debug.h"

void postureBegin();          // 初始化 I2C 与传感器，并记录上电时的基准姿态
void postureUpdate();         // 周期调用：每 POSTURE_READ_MS 读一次
void posturePrintReport();    // 打印一行数据（标定阶段用，看串口就能采数据）

bool  postureReady();         // 传感器是否就绪

/* ---------------- 原始量与派生量 ---------------- */
float postureAccelG();        // 合加速度（g），静止时约 1.0
float posturePitchDeg();      // 俯仰角（度）
float postureRollDeg();       // 横滚角（度）
float postureTiltFromBase();  // 相对上电基准的倾角（度）—— 判断"翻没翻"
float postureMotionLevel();   // 运动强度：半秒内合加速度的峰峰值（g）
float postureTemperature();   // 片上温度（摄氏度）

/* ---------------- 判据 ---------------- */
bool  postureImpactSeen();    // 上电以来见过"失重→冲击"的入水波形没有
bool  postureImpactRecent();  // 判据①：冲击是最近 POSTURE_IMPACT_KEEP_MS 内发生的
bool  postureFlipped();       // 判据②：相对上电基准转过 POSTURE_TILT_MIN_DEG 以上
bool  postureCalm();          // 判据③：运动小于 POSTURE_CALM_MAX_G（平静 = 漂着）
float postureSwayHz();        // 摇摆频率（Hz）；窗口没填满返回 -1。只用于打印

bool  postureLooksLikeFloating();   // 三条判据综合

/* 演示用：强制让三条判据都通过（按 BOOT 键模拟不了"失重→冲击"的波形） */
void  postureSetSim(bool on);
bool  postureSimForced();

#endif
