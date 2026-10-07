/* =====================================================================
   voice.cpp  SYN6288 组帧与播报实现
   ---------------------------------------------------------------------
   帧格式：0xFD + 长度(2) + 0x01 + 0x00 + GB2312文本 + 异或校验(1)
   校验必须从帧头 0xFD 开始异或。
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
   新句子只有优先级更高时才允许打断，否则就得等 —— 这样
   「落水告警」能顶掉靠泊距离播报，而靠泊播报顶不掉落水。 */
static uint32_t s_busyUntilMs = 0;
static uint8_t  s_busyPrio    = 0;

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

/* 估这句话念完要多久：SYN6288 默认语速约每秒 4 个字，也就是一个字 250 毫秒。
   汉字算 1 个字；GB2312 标点（A3xx，比如逗号）算半个字，它只是换口气；
   ASCII 的数字算 1 个字（"1.8" 会念成"一点八"，差不多对得上）；
   开头的音量标签 [v16] 模块不念，跳过。
   最后整体再加三成余量和 0.4 秒垫底 —— 宁可多等一会儿，也别把话掐了。 */
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

  /* 一个字按 300 毫秒算（原来 250），再垫 1 秒（原来 0.4 秒）。
     实测 250 毫秒估得偏短：数字和标点的停顿比按字数算出来的长，
     结果下一句还是抢在前头。宁可多等一会儿，也别把话掐了。      */
  uint32_t ms = units10 * 30;
  return ms + 1000;
}

/* prio：这句话的分量，默认按"告警级"算，距离播报和落水另行指定 */
static void voiceSend(uint8_t prio = VOICE_PRIO_ALARM) {
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

/* 现在还有话没念完吗？（按估算的时长判断，误差半秒左右）
   想插话的模块先来问这一句：靠泊距离播报问到"忙"就跳过这一轮，
   等下一轮报最新的距离；告警则用下面那个问"正在念的是不是告警"。 */
bool voiceBusy() {
  return (int32_t)(s_busyUntilMs - millis()) > 0;
}

/* 正在念的那句有多重要（0 = 没人念）：
   想插话的模块拿它跟自己的优先级比，比自己高就得让，比自己低就能打断。 */
uint8_t voiceBusyPrio() {
  return voiceBusy() ? s_busyPrio : 0;
}

void voiceSpeakTest() {
  txtReset();
  txtAddVolume();
  VADD(GB_HELLO);
  voiceSend();
}

void voiceSpeakStartup() {
  txtReset();
  txtAddVolume();
  VADD(GB_STARTUP);
  voiceSend();
}

void voiceAnnounce(bool haveDir, double tLat, double tLon, float distM, int sector, bool useRel) {
  txtReset();
  txtAddVolume();
  VADD(GB_FELL_OVERBOARD);

  if (haveDir) {
    // 绝对方位：“人员落水，信标在 东北 方向，距离约 1234 米，坐标…”
    // 相对船头：“人员落水，信标在 左前方，距离约 1234 米，坐标…”
    VADD(GB_BEACON_AT);
    if (useRel) {
      VADD(GB_REL_DIR[sector & 7]);
    } else {
      VADD(GB_DIR[sector & 7]);
      VADD(GB_DIRECTION);
    }
    VADD(GB_DIST_ABOUT);
    if (distM < 1000.0f) {
      txtAddNum(distM, 0);
      VADD(GB_METER);
    } else {
      txtAddNum(distM / 1000.0f, 1);
      VADD(GB_KILOMETER);
    }
  } else {
    VADD(GB_RECV_COORD);
  }

  txtAddLat(tLat);
  txtAddLon(tLon);
  if (!haveDir) VADD(GB_SELF_NOPOS);
  voiceSend(VOICE_PRIO_SOS);    // 人员落水：最高优先级，可以顶掉靠泊播报
}

void voiceSpeakTargetNoPos() {
  txtReset();
  txtAddVolume();
  VADD(GB_TARGET_NOPOS);
  voiceSend();
}

void voiceSpeakLinkLost() {
  txtReset();
  txtAddVolume();
  VADD(GB_LINK_LOST);
  voiceSend(VOICE_PRIO_SOS);    // 信标失联：搜救相关，同样最高
}

void voiceSpeakLinkBack() {
  txtReset();
  txtAddVolume();
  VADD(GB_LINK_BACK);
  voiceSend(VOICE_PRIO_SOS);
}

/* ---------------- 靠泊辅助 ---------------- */

// 距离怎么念：1 米以上念“距离三点二米”，1 米以下换算成厘米念“距离八十厘米”
static void txtAddDistance(float d) {
  VADD(GB_DISTANCE);
  if (d >= 1.0f) {
    txtAddNum(d, 1);
    VADD(GB_METER);
  } else {
    txtAddNum(d * 100.0f, 0);
    VADD(GB_CENTIMETER);
  }
}

/* 先说这一句是哪一路测出来的：右舷那只念“右侧”，船头那只念“前方”。
   两路说的是同一件事，靠泊时人不用回头看屏幕也能分清方向。 */
static void txtAddSource(bool side) {
  VADD(side ? GB_STARBOARD : GB_AHEAD);
}

void voiceSpeakBerthEnter(float distM, bool side) {
  txtReset();
  txtAddVolume();
  VADD(GB_BERTH_WATCH);
  txtAddSource(side);
  txtAddDistance(distM);
  voiceSend();
}

void voiceSpeakBerthDistance(float distM, bool soon, bool side) {
  txtReset();
  txtAddVolume();
  txtAddSource(side);
  txtAddDistance(distM);
  if (soon) VADD(GB_SOON_DOCK);
  voiceSend(VOICE_PRIO_DIST);   // 靠泊距离播报：优先级最低，谁都能顶掉它
}

void voiceSpeakBerthAlarm(uint8_t code, bool side) {
  txtReset();
  txtAddVolume();
  switch (code) {
    case 0x01: txtAddSource(side); VADD(GB_ALM_SPEED_HI);  break;
    case 0x02: txtAddSource(side); VADD(GB_ALM_SPEED_MAX); break;
    case 0x03: txtAddSource(side); VADD(GB_ALM_TOO_NEAR);  break;
    default: return;
  }
  voiceSend(VOICE_PRIO_ALARM);  // 告警：能顶掉距离播报，顶不掉落水
}

void voiceSpeakBerthDone(bool side) {
  txtReset();
  txtAddVolume();
  txtAddSource(side);
  VADD(GB_BERTH_DONE);
  voiceSend();
}

/* ---------------- 走锚监测 ---------------- */

void voiceSpeakAnchorOn() {
  txtReset();
  txtAddVolume();
  VADD(GB_ANCHOR_ON);
  voiceSend();
}

void voiceSpeakAnchorSuspect(float driftM, int sector) {
  txtReset();
  txtAddVolume();
  VADD(GB_ANCHOR_SUSPECT);
  txtAddNum(driftM, 1);
  VADD(GB_METER);
  VADD(GB_DRIFT_DIR);
  VADD(GB_DIR[sector & 7]);
  voiceSend();
}

void voiceSpeakAnchorDragging() {
  txtReset();
  txtAddVolume();
  VADD(GB_ANCHOR_DRAG);
  voiceSend();
}

void voiceSpeakAnchorOk() {
  txtReset();
  txtAddVolume();
  VADD(GB_ANCHOR_OK);
  voiceSend();
}
