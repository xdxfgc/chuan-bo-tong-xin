/* =====================================================================
   logbook.h   数据记录与回放（黑匣子）
   ---------------------------------------------------------------------
   每秒把一帧系统状态压进 RAM 环形缓冲，之后可以：
     · 串口 dump 出完整 CSV
     · 网页下载 CSV（带表头，Excel 直接能开）
     · 网页按时间轴回放
   只占内存、不碰 flash——flash 每秒写一次会磨损。
   ===================================================================== */

#ifndef CB_LOGBOOK_H
#define CB_LOGBOOK_H

#include "config.h"

void logbookBegin();
void logbookUpdate();            // 周期调用：到点存一条

bool     logbookOn();            // 是否正在记录
void     logbookSetOn(bool on);  // 开始 / 暂停记录
void     logbookClear();         // 清空

uint16_t logbookCount();         // 已有多少条
uint16_t logbookCapacity();      // 容量
uint32_t logbookSpanSec();       // 这批记录覆盖多少秒

String   logbookHeader();        // CSV 表头
String   logbookLine(uint16_t k);// 第 k 条（0 = 最旧）的 CSV 行
String   logbookCSV();           // 完整 CSV（网页下载用）

void     logbookPrintStatus();   // 串口打印状态

#endif
