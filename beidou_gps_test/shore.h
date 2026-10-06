/* =====================================================================
   shore.h   岸基节点跟踪（"距岸基 / 岸基方位"）
   ---------------------------------------------------------------------
   岸基节点固定在码头上，每 2 秒广播一条 R 参考帧，里面是它自己的定位。
   船端收到之后，用本船北斗位置算出"距岸基多远、在哪个方向"。

   为什么要有这个：
     靠泊最后四米靠激光测距，但前面几十米激光看不到。有了岸基位置，
     船在远处就知道自己离码头还有多远，可以提前减速。

   岸基的两种帧都能用：
     R —— 参考站广播（主用）
     A —— 岸基回信标的应答，里面也带着岸基自己的位置（R 丢了时兜底）

   注意：岸基只是"参考点"，不做告警、不播报 —— 免得码头一直响。
   ===================================================================== */

#ifndef CB_SHORE_H
#define CB_SHORE_H

#include "config.h"
#include "lora_link.h"

void shoreBegin();                          // 初始化
void shoreOnPacket(const TargetPacket& pkt);// 收到岸基 R/A 帧后调用
void shoreUpdate();                         // 周期调用：刷新方位、判断链路超时
void shorePrintReport();                    // 串口打印一行

bool        shoreLinkUp();                  // 岸基链路是否在线
bool        shoreHas();                     // 是否收到过岸基数据
bool        shoreValid();                   // 岸基定位是否有效
int         shoreId();                      // 岸基编号（21/22…），0 = 老格式没带编号
uint32_t    shoreSeq();                     // 最近一包的序号
double      shoreLat();
double      shoreLon();
int         shoreRssi();
float       shoreSnr();
bool        shoreHaveDir();                 // 本船已定位且岸基坐标有效
float       shoreDistM();                   // 到岸基的距离（米）
float       shoreBearing();                 // 岸基方位角（度）
const char* shoreDirText();                 // 方位文字，如“东北”

#endif
