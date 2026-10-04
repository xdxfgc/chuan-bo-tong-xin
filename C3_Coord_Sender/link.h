/* =====================================================================
   link.h   一问一答的链路层
   ---------------------------------------------------------------------
   管“一轮”的事：把一帧发出去，等应答；收不到就重传，最多 MAX_RETRY 次。
   顺便统计成功轮数、重发次数、失败轮数——这些数据写试验报告时用得上。
   ===================================================================== */

#ifndef CB_LINK_H
#define CB_LINK_H

#include "config.h"
#include "debug.h"

// 发一帧并等应答。返回是否成功；ackOut 传出收到的应答原文，attemptsUsed 传出用了几次。
bool linkSendWithAck(const char* frame, String& ackOut, int& attemptsUsed);

void linkPrintStats();     // 打印统计

#endif
