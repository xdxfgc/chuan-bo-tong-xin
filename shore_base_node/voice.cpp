/* =====================================================================
   voice.cpp   岸基节点 SYN6288 组帧与播报实现
   ---------------------------------------------------------------------
   帧格式：0xFD + 长度(2) + 0x01 + 0x00 + GB2312文本 + 异或校验(1)
   校验必须从帧头 0xFD 开始异或。

   和船端 voice.cpp 是同一套实现，去掉了靠泊、走锚、相对船头
   这些岸基节点用不到的播报。
   ===================================================================== */

#include "voice.h"
#include "gb2312_voice.h"

#define VOICE_TEXT_MAX 128

static HardwareSerial SynSerial(2);       // UART1 被北斗占用，语音用 UART2

static uint8_t s_text[VOICE_TEXT_MAX];
static size_t  s_len = 0;
static uint8_t s_volume = 16;

/* 一句话要念多久 —— 发出去之后记在这里，谁想插话先来问 voiceBusy()。
   s_busyPrio 记的是"正在念的那句有多大分量"（见 voice.h 的三个优先级）：
   新句子只有优先级更高时才允许打断，否则就得让路。
   和船端是同一套实现，两边行为保持一致。                        */
static uint32_t s_busyUntilMs = 0;
static uint8_t  s_busyPrio    = 0;

/* 语音状态（网页显示用） */
static const char* s_lastLabel = "";
static uint32_t    s_lastMs    = 0;
static uint32_t    s_count     = 0;
static uint32_t    s_skipCount = 0;

/* ---------------- 文本拼装 ---------------- */

#define VADD(frag) txtAdd((frag), sizeof(frag) - 1)     // 自动去掉结尾的 0x00

static void txtReset() { s_len = 0; }

static void txtAdd(const uint8_t* p, size_t n) {
  if (s_len + n > sizeof(s_text)) n = sizeof(s_text) - s_len;
  memcpy(s_text + s_len, p, n);
  s_len += n;
}

static void txtAddAscii(const char* s) { txtAdd((const uint8_t*)s, strlen(s)); }

// 数字按 ASCII 发过去，SYN6288 会按中文念出来
static void txtAddNum(double v, int decimals) {
  char b[24];
  snprintf(b, sizeof(b), "%.*f", decimals, v);
  txtAddAscii(b);
}

// 音量标签 [v16]，放在文本最前面
static void txtAddVolume() {
  char b[8];
  snprintf(b, sizeof(b), "[v%d]", (int)s_volume);
  txtAddAscii(b);
}

static void txtAddLat(double lat) {
  VADD(GB_COORD);
  VADD(lat >= 0 ? GB_NORTH : GB_SOUTH);
  txtAddNum(fabs(lat), 4);
  VADD(GB_DEGREE);
}

static void txtAddLon(double lon) {
  VADD(GB_COORD);
  VADD(lon >= 0 ? GB_EAST : GB_WEST);
  txtAddNum(fabs(lon), 4);
  VADD(GB_DEGREE);
}

/* ---------------- 发送一帧 ---------------- */

/* 估这句话念完要多久：SYN6288 默认语速约每秒 4 个字，一个字 300 毫秒，
   再垫 1 秒余量（和船端一致 —— 宁可多等一会儿，也别把话掐了）。 */
static uint32_t estimateSpeechMs() {
  uint32_t units10 = 0;                 // 以 0.1 个字为单位数，避免浮点
  size_t   i = 0;

  if (s_len >= 2 && s_text[0] == '[') { // 跳过开头的 [v16]
    while (i < s_len && s_text[i] != ']') i++;
    if (i < s_len) i++;
  }

  for (; i < s_len; i++) {
    if (s_text[i] == 0xA3 && i + 1 < s_len) { units10 += 5;  i++; }   // 中文标点
    else if (s_text[i] < 0x80)              { units10 += 10; }        // 数字、小数点
    else                                    { units10 += 10; i++; }   // 汉字（两字节）
  }

  return units10 * 30 + 1000;
}

/* prio  ：这句话的分量；label：这句话叫什么（只给网页显示用） */
static void voiceSend(uint8_t prio = VOICE_PRIO_ALARM, const char* label = "") {
  if (s_len == 0) return;

  uint8_t  frame[VOICE_TEXT_MAX + 8];
  uint16_t dataLen = (uint16_t)s_len + 3;      // 命令字1 + 参数1 + 文本n + 校验1

  frame[0] = 0xFD;
  frame[1] = (dataLen >> 8) & 0xFF;
  frame[2] = dataLen & 0xFF;
  frame[3] = 0x01;                             // 命令字：合成播放
  frame[4] = 0x00;                             // 文本编码：0 = GB2312
  memcpy(frame + 5, s_text, s_len);

  uint8_t xorSum = 0;
  for (size_t i = 0; i < 5 + s_len; i++) xorSum ^= frame[i];
  frame[5 + s_len] = xorSum;

  SynSerial.write(frame, 6 + s_len);
  SynSerial.flush();

  // 模块收到合法命令会回 0x41；读掉它，免得缓冲区越堆越多
  delay(20);
  int ack = -1;
  while (SynSerial.available()) ack = SynSerial.read();

  /* 记住"念到什么时候"，顺便把估算值打出来，方便现场对语速 */
  uint32_t ms = estimateSpeechMs();
  s_busyPrio    = prio;
  s_busyUntilMs = millis() + ms;
  s_lastLabel   = (label && *label) ? label : "播报";
  s_lastMs      = millis();
  s_count++;

  Serial.printf("[语音] 发送 %u 字节，预计念 %.1f 秒%s\n", (unsigned)s_len, ms / 1000.0,
                ack < 0 ? "" : (ack == 0x41 ? "  模块应答 OK" : "  应答异常"));
}

/* ---------------- 对外接口 ---------------- */

void voiceBegin() {
  SynSerial.begin(SYN_BAUD, SERIAL_8N1, SYN_RX_PIN, SYN_TX_PIN);
  s_volume = (SYN_VOLUME > 16) ? 16 : SYN_VOLUME;
}

void voiceSetVolume(uint8_t v) {
  s_volume = (v > 16) ? 16 : v;
  Serial.printf("[语音] 音量已设为 %u/16\n", (unsigned)s_volume);
}

uint8_t voiceVolume() { return s_volume; }

bool voiceBusy() {
  return (int32_t)(s_busyUntilMs - millis()) > 0;
}

uint8_t voiceBusyPrio() {
  return voiceBusy() ? s_busyPrio : 0;
}

uint32_t voiceBusyLeftMs() {
  int32_t left = (int32_t)(s_busyUntilMs - millis());
  return (left > 0) ? (uint32_t)left : 0;
}

const char* voiceLastLabel() { return (s_lastLabel && *s_lastLabel) ? s_lastLabel : "--"; }
uint32_t    voiceLastMs()     { return s_lastMs; }
uint32_t    voiceCount()      { return s_count; }
uint32_t    voiceSkipCount()  { return s_skipCount; }
void        voiceNoteSkip()   { s_skipCount++; }

void voiceSpeakTest() {
  txtReset();
  txtAddVolume();
  VADD(GB_HELLO);
  voiceSend(VOICE_PRIO_ALARM, "试听 你好");
}

void voiceSpeakStartup() {
  txtReset();
  txtAddVolume();
  VADD(GB_STARTUP);
  voiceSend(VOICE_PRIO_ALARM, "开机提示");
}

void voiceAnnounce(bool haveDir, double tLat, double tLon, float distM, int sector) {
  txtReset();
  txtAddVolume();
  VADD(GB_FELL_OVERBOARD);

  if (haveDir) {
    // “人员落水，信标在 东北 方向，距离约 1234 米，坐标…”
    VADD(GB_BEACON_AT);
    VADD(GB_DIR[sector & 7]);
    VADD(GB_DIRECTION);
    VADD(GB_DIST_ABOUT);
    if (distM < 1000.0f) {
      txtAddNum(distM, 0);
      VADD(GB_METER);
    } else {
      txtAddNum(distM / 1000.0f, 1);
      VADD(GB_KILOMETER);
    }
  } else {
    // 本节点还没定位：“人员落水，收到坐标…”
    VADD(GB_RECV_COORD);
  }

  txtAddLat(tLat);
  txtAddLon(tLon);
  if (!haveDir) VADD(GB_SELF_NOPOS);
  voiceSend(VOICE_PRIO_SOS, "人员落水");
}

void voiceSpeakTargetNoPos() {
  txtReset();
  txtAddVolume();
  VADD(GB_TARGET_NOPOS);
  voiceSend(VOICE_PRIO_ALARM, "信标未定位");
}

void voiceSpeakLinkLost() {
  txtReset();
  txtAddVolume();
  VADD(GB_LINK_LOST);
  voiceSend(VOICE_PRIO_SOS, "信标失联");
}

void voiceSpeakLinkBack() {
  txtReset();
  txtAddVolume();
  VADD(GB_LINK_BACK);
  voiceSend(VOICE_PRIO_SOS, "通信恢复");
}
