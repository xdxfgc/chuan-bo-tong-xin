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

/* ---------------- 播报优先级 ----------------
   和船端同一套：数字大的可以打断数字小的，小的必须等或干脆让路。
   岸基没有靠泊播报，所以实际只用到 2 和 3 两档。            */
#define VOICE_PRIO_DIST   1     // 保留（岸基用不到）
#define VOICE_PRIO_ALARM  2     // 信标未定位、试听、开机提示
#define VOICE_PRIO_SOS    3     // 人员落水、信标失联/恢复：最高

/* ---------------- 语音状态（给网页显示用） ---------------- */
bool        voiceBusy();           // 还有话没念完（按字数估算）
uint8_t     voiceBusyPrio();       // 正在念的那句的优先级（0 = 没在念）
uint32_t    voiceBusyLeftMs();     // 预计还要念多久（毫秒）
const char* voiceLastLabel();      // 最近一次播报的名称（UTF-8）
uint32_t    voiceLastMs();         // 最近一次播报的时刻（millis）
uint32_t    voiceCount();          // 累计播报条数
uint32_t    voiceSkipCount();      // 因为"上一句没念完"被跳过的次数
void        voiceNoteSkip();       // 调用方跳过一条播报时记一次

#endif
