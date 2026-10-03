/* =====================================================================
   voice.h   SYN6288 语音播报
   ---------------------------------------------------------------------
   接线：ESP32 GPIO16(TX) -> 模块 RXD，GPIO4(RX) <- 模块 TXD（可选），
         模块 VCC -> 5V，GND 共地，喇叭接 SPK+/SPK-。
   ===================================================================== */

#ifndef CB_VOICE_H
#define CB_VOICE_H

#include "config.h"

void voiceBegin();                 // 初始化语音串口

// 音量：0~16，开机取 config.h 里的 SYN_VOLUME；随时可改，下一句就生效
void    voiceSetVolume(uint8_t v);
uint8_t voiceVolume();

void voiceSpeakTest();             // 播一句“你好”，调音量试听用
void voiceSpeakStartup();          // 开机提示音

// 播报信标：方向 + 距离 + 坐标。haveDir=false 时只报坐标并说明本方未定位
void voiceAnnounce(bool haveDir, double tLat, double tLon, float distM, int sector);

void voiceSpeakTargetNoPos();      // 信标未定位
void voiceSpeakLinkLost();         // 与信标失去联系
void voiceSpeakLinkBack();         // 通信已恢复

// 串口命令：在串口监视器里输入 v0~v16 回车，在线调音量
void voicePollSerial();

#endif
