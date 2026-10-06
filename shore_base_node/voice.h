/* =====================================================================
   voice.h   岸基节点 SYN6288 语音播报
   ---------------------------------------------------------------------
   接线（和船端完全一致）：
     ESP32 GPIO16(TX) -> 模块 RXD（必接）
     ESP32 GPIO4 (RX) <- 模块 TXD（可选，用来读应答 0x41）
     模块 VCC -> 5V，GND 共地，喇叭接 SPK+/SPK-。

   岸基节点只管两件事：信标落水播报、与信标失联/恢复播报。
   没有磁力计也没有船头方向，所以方位一律用绝对方位（东北这种）。
   ===================================================================== */

#ifndef SB_VOICE_H
#define SB_VOICE_H

#include "config.h"

void voiceBegin();                 // 初始化语音串口

// 音量：0~16，开机取 config.h 里的 SYN_VOLUME；随时可改，下一句就生效
void    voiceSetVolume(uint8_t v);
uint8_t voiceVolume();

void voiceSpeakTest();             // 播一句“你好”，调音量试听用
void voiceSpeakStartup();          // 开机提示音

// 播报落水信标：绝对方位 + 距离 + 坐标。
// haveDir=false（本节点还没定位）时只报坐标并说明本方未定位。
// sector 用 track.cpp 的绝对方位扇区：0 正北 1 东北 … 7 西北。
void voiceAnnounce(bool haveDir, double tLat, double tLon, float distM, int sector);

void voiceSpeakTargetNoPos();      // 信标未定位
void voiceSpeakLinkLost();         // 与信标失去联系
void voiceSpeakLinkBack();         // 通信已恢复

#endif