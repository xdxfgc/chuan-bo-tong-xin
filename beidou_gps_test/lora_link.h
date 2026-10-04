/* =====================================================================
   lora_link.h   LoRa 链路（SX1278 433MHz）
   ---------------------------------------------------------------------
   注意：文件名不能用 lora.h。Windows 不区分文件名大小写，
   `#include <LoRa.h>`（库）会优先匹配到同目录的 lora.h，导致库加载失败。

   协议（和信标端 ESP32-C3、岸基约定，三端必须一致）：
     信标 -> 本板（新）：M,<信标ID>,<序号>,P,<定位有效 0/1>,<纬度>,<经度>
     信标 -> 本板（老）：M,<序号>,P,<定位有效 0/1>,<纬度>,<经度>
     本板 -> 信标：      A,<船ID>,<序号>,C,<本方定位有效 0/1>,<纬度>,<经度>
     本板 -> 岸基：      S,<船ID>,<序号>,P,<本方定位有效 0/1>,<纬度>,<经度>,
                         <对地速度节>,<航向度>,<卫星数>

   入帧的新老两种格式都认：老格式第 3 个字段是 "P"，新格式第 2 个字段是信标 ID。
   老格式帧的 srcId 记为 0。同一只发送者、同一序号的重传包会被标记为 duplicate，
   只回 ACK，不重复显示和播报——按发送者分开记，多只信标同时工作不会互相误判。
   ===================================================================== */

#ifndef CB_LORA_LINK_H
#define CB_LORA_LINK_H

#include "config.h"

struct TargetPacket {
  int      srcId     = 0;       // 发送者标识（0 = 老格式，没有 ID）
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
// 船端定期广播自己的状态（岸基节点收这个）
bool loraSendShipStatus(bool valid, double lat, double lon,
                        float sogKnots, float cog, int sats);
int  loraChannelRssi();           // 信道底噪，用于判断射频是否正常工作

#endif
