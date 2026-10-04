/* =====================================================================
   radio.h   LoRa 收发底层（SX1278 433MHz）
   ---------------------------------------------------------------------
   只管“发一帧”和“收一帧”，不管重传、不管协议内容。
   用非阻塞发送 + 轮询中断标志，所以 DIO0 不用接线。
   ===================================================================== */

#ifndef CB_RADIO_H
#define CB_RADIO_H

#include "config.h"
#include "debug.h"

bool radioBegin();                                   // 初始化射频，失败返回 false
bool radioIsReady();

bool radioSend(const char* s);                       // 发一帧
bool radioReceive(String& out, uint32_t timeoutMs);  // 在超时内等一帧

int   radioLastRssi();                               // 最近一帧的 RSSI
float radioLastSnr();                                // 最近一帧的 SNR

#endif
