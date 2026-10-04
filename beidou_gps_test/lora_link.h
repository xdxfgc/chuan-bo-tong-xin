/* =====================================================================
   lora_link.h   LoRa 链路（SX1278 433MHz）
   ---------------------------------------------------------------------
   注意：文件名不能用 lora.h。Windows 不区分文件名大小写，
   `#include <LoRa.h>`（库）会优先匹配到同目录的 lora.h，导致库加载失败。

   协议（和信标端 ESP32-C3、岸基约定，三端必须一致）：
     信标 -> 本板（新）：M,<信标ID>,<序号>,P,<定位有效 0/1>,<纬度>,<经度>
     信标 -> 本板（老）：M,<序号>,P,<定位有效 0/1>,<纬度>,<经度>
     本板 -> 信标：      A,<船ID>,<序号>,C,<本方定位有效 0/1>,<纬度>,<经度>
     本板 -> 岸基：      S,<船ID>,<序号>,P,<本方定位有效 0/1>,<纬度>,<经度>

   入帧的新老两种格式都认：老格式第 3 个字段直接就是 "P"，新格式第 2 个字段
   是信标 ID。信标 ID 用 11/12/13……（本板是 1），老格式帧的 ID 记为
   BEACON_ID_NONE。同一只信标、同一序号的重传包会被标记为 duplicate，
   只回 ACK，不重复显示和播报。

   末尾的 A 帧（应答）和 S 帧（本船广播）由 selfcast 模块按 BOAT_BCAST_MS 发。
   ===================================================================== */

#ifndef CB_LORA_LINK_H
#define CB_LORA_LINK_H

#include "config.h"

#define BEACON_ID_NONE 0        // 0 = 该帧没带发送者编号（老格式，或是岸基广播）

struct TargetPacket {
  uint8_t  beaconId  = BEACON_ID_NONE;  // 发送者编号，信标 11/12/13……
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
bool loraSendText(const char* s); // 直接发一帧原文（定期广播用），成功返回 true
int  loraChannelRssi();           // 信道底噪，用于判断射频是否正常工作

#endif
