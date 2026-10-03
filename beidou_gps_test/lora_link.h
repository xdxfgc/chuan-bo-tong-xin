/* =====================================================================
   lora_link.h   LoRa 链路（SX1278 433MHz）
   ---------------------------------------------------------------------
   注意：文件名不能用 lora.h。Windows 不区分文件名大小写，
   `#include <LoRa.h>`（库）会优先匹配到同目录的 lora.h，导致库加载失败。

   协议（和信标端 ESP32-C3 约定，两端必须一致）：
     信标 -> 本板：  M,<序号>,P,<定位有效 0/1>,<纬度>,<经度>
     本板 -> 信标：  A,<序号>,C,<本方定位有效 0/1>,<纬度>,<经度>
   同一序号的重传包会被标记为 duplicate，只回 ACK，不重复显示和播报。
   ===================================================================== */

#ifndef CB_LORA_LINK_H
#define CB_LORA_LINK_H

#include "config.h"

struct TargetPacket {
  uint32_t seq       = 0;
  bool     valid     = false;   // 信标定位是否有效
  double   lat       = 0.0;
  double   lon       = 0.0;
  bool     duplicate = false;   // 与上一包同序号（信标没收到 ACK 重发的）
  int      rssi      = 0;
  float    snr       = 0.0f;
};

bool loraBegin();                 // 初始化射频，失败返回 false
bool loraIsReady();
bool loraPoll(TargetPacket* out); // 收到一包返回 true
void loraSendAck(uint32_t seq, bool centerValid, double lat, double lon);
int  loraChannelRssi();           // 信道底噪，用于判断射频是否正常工作

#endif
