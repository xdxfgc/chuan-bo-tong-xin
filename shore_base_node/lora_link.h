/* =====================================================================
   lora_link.h   岸基节点 LoRa 链路（SX1278 433MHz）
   ---------------------------------------------------------------------
   注意：文件名不能用 lora.h。Windows 不区分文件名大小写，
   `#include <LoRa.h>`（库）会优先匹配到同目录的 lora.h，导致库加载失败。

   协议（和船端 beidou_gps_test、信标 C3_Coord_Sender 三端一致）：

     信标 -> 所有： M,<信标ID>,<序号>,P,<定位有效 0/1>,<纬度>,<经度>
     船端 -> 所有： S,<船ID>,<序号>,P,<定位有效 0/1>,<纬度>,<经度>,
                      <对地速度节>,<航向度>,<卫星数>
     本节点 -> 所有：R,<岸基ID>,<序号>,P,<本节点定位有效 0/1>,<纬度>,<经度>
     应答 -> 对方： A,<自己ID>,<序号>,C,<自己定位有效 0/1>,<纬度>,<经度>

   为什么本节点用 R 不用 S：船端已经占了 S，两个都用 S 会分不清。
   R = Reference（参考站）。

   老格式（第 2 个字段直接是序号、没有编号）也认，见 lora_link.cpp。
   ===================================================================== */

#ifndef SB_LORA_LINK_H
#define SB_LORA_LINK_H

#include "config.h"

/* 帧的角色 */
enum LoraKind {
  LK_NONE   = 0,   // 无效
  LK_BEACON = 'M', // 信标
  LK_VESSEL = 'S', // 船端
  LK_SHORE  = 'R', // 岸基（别的岸基节点发来的，本节点只忽略）
  LK_ACK    = 'A'  // 应答（里面带着发送者的位置，可当作对方的位置）
};

struct LoraPacket {
  LoraKind kind      = LK_NONE;
  int      srcId     = 0;       // 发送者编号；0 = 老格式帧，没带编号
  uint32_t seq       = 0;
  bool     valid     = false;   // 对方定位是否有效
  double   lat       = 0.0;
  double   lon       = 0.0;

  /* 附加字段：只有船端的 S 帧带（对地速度、航向、卫星数）。
     信标帧没有这几个，hasExtra 就是 false。                        */
  bool     hasExtra  = false;
  float    sogKnots  = 0.0f;    // 对地速度（节）
  float    cogDeg    = 0.0f;    // 对地航向（度）
  int      sats      = 0;       // 参与定位的卫星数

  bool     duplicate = false;   // 同一个发送者、同一个序号（对方没收到 ACK 重发的）
  int      rssi      = 0;
  float    snr       = 0.0f;
};

bool loraBegin();                 // 初始化射频，失败返回 false
bool loraIsReady();
bool loraPoll(LoraPacket* out);   // 收到一包返回 true（kind 区分信标/船端）
bool loraSendText(const char* s); // 通用文本发送（内部等 TxDone，成功返回 true）
void loraSendAck(int seq, bool myValid, double lat, double lon);
int  loraChannelRssi();           // 信道底噪，用于判断射频是否正常工作

#endif
