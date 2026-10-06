/* =====================================================================
   coord.h   坐标来源
   ---------------------------------------------------------------------
   四种来源（在 config.h 里用 SRC_MODE 切换）：
     0 固定坐标 / 1 绕点转圈 / 2 本端北斗 / 3 串口手动输入
   对外只提供三个接口：初始化、周期轮询、取当前坐标。
   主程序不关心坐标是哪来的，换来源只改 config.h。
   ===================================================================== */

#ifndef CB_COORD_H
#define CB_COORD_H

#include "config.h"
#include "debug.h"

void coordBegin();                 // 初始化（模式 2 会打开北斗串口）
void coordPoll();                  // 周期调用：模式 2/3 需要读串口
void coordGet(bool* valid, double* lat, double* lon);   // 取当前坐标

const char* coordSourceText();     // 坐标来源的文字说明，开机打印用

// 组出要发送的载荷：P,<定位有效>,<纬度>,<经度>
void coordBuildPayload(char* buf, size_t n);

/* 让北斗模块进待机 / 唤醒（只有 SRC_MODE = 2 有意义，其它模式是空操作）。
   待机电流比正常工作低一些，但**到不了 200µA** —— 真要省电还是得给
   北斗模块的 VCC 串一个 MOS 开关（见 config.h 的 GPS_PWR_PIN）。      */
void coordSleep();
void coordWake();

#endif
