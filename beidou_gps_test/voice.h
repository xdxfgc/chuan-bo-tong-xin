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

/* ---------------- 播报优先级 ----------------
   一句话念着的时候，谁有资格插话由优先级决定：数字大的可以打断数字小的，
   数字小的必须等（或者干脆跳过）。落水永远是老大。                  */
#define VOICE_PRIO_DIST   1     // 靠泊距离播报：最低，永远让路
#define VOICE_PRIO_ALARM  2     // 靠泊告警、靠妥、进入监测、走锚
#define VOICE_PRIO_SOS    3     // 人员落水、信标失联/恢复：最高

/* 播报排队用：这一句还没念完就先别开口，免得把上一句掐断。
   voiceBusy()      —— 还有话没念完（按字数估算）
   voiceBusyPrio()  —— 正在念的那句的优先级（0 = 没在念） */
bool    voiceBusy();
uint8_t voiceBusyPrio();

void voiceSpeakTest();             // 播一句“你好”，调音量试听用
void voiceSpeakStartup();          // 开机提示音

// 播报信标：方向 + 距离 + 坐标。haveDir=false 时只报坐标并说明本方未定位。
// useRel=true 用「相对船头」的八个方位（左前方这种），false 用绝对方位（东北方向这种）。
void voiceAnnounce(bool haveDir, double tLat, double tLon, float distM, int sector, bool useRel);

void voiceSpeakTargetNoPos();      // 信标未定位
void voiceSpeakLinkLost();         // 与信标失去联系
void voiceSpeakLinkBack();         // 通信已恢复

/* ---------------- 靠泊辅助的播报 ---------------- */
/* side = 这一句用的是哪一路激光：true 右舷那只（念“右侧”），false 船头那只（念“前方”）。
   两路播报词一样，只有开头的方位词不同，这样听声音就知道是哪一侧在报数。 */
void voiceSpeakBerthEnter(float distM, bool side);              // “靠泊监测，前方距离三米四”
void voiceSpeakBerthDistance(float distM, bool soon, bool side);// “右侧距离一米八” / “前方距离三十厘米，即将靠妥”
void voiceSpeakBerthAlarm(uint8_t code, bool side);             // “前方靠泊速度偏大，请减速”
void voiceSpeakBerthDone(bool side);                            // “前方靠泊完成” / “右侧靠泊完成”

/* ---------------- 走锚监测的播报 ---------------- */
void voiceSpeakAnchorOn();                           // “锚泊监测已启动，基准位置已记录”
void voiceSpeakAnchorSuspect(float driftM, int sector); // “疑似走锚，位移2.4米，漂移方向东南”
void voiceSpeakAnchorDragging();                     // “船正在走锚，请立即处理”
void voiceSpeakAnchorOk();                           // “位移已回到正常范围”

#endif
