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

static void voiceSend() {
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
  Serial.printf("[语音] 发送 %u 字节%s\n", (unsigned)s_len,
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

void voiceAnnounce(bool haveDir, double tLat, double tLon, float distM, int sector) {
  txtReset();
  txtAddVolume();
  VADD(GB_FELL_OVERBOARD);

  if (haveDir) {
    // “人员落水，信标在 东北 方向，距离约 1234 米，坐标 北纬31.2304度…”
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
    VADD(GB_RECV_COORD);
  }

  txtAddLat(tLat);
  txtAddLon(tLon);
  if (!haveDir) VADD(GB_SELF_NOPOS);
  voiceSend();
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
  voiceSend();
}

void voiceSpeakLinkBack() {
  txtReset();
  txtAddVolume();
  VADD(GB_LINK_BACK);
  voiceSend();
}

// 串口命令：v10 / vol=10 / 10 都能识别
void voicePollSerial() {
  static char   buf[16];
  static size_t n = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) {
        buf[n] = '\0';
        const char* p = buf;
        while (*p && !isdigit((unsigned char)*p)) p++;
        if (*p) {
          int v = atoi(p);
          if (v > 16) v = 16;
          if (v < 0)  v = 0;
          voiceSetVolume((uint8_t)v);
          voiceSpeakTest();
        } else {
          Serial.println("用法：输入 v0~v16 回车，例如 v10（0 最小，16 最大）");
        }
        n = 0;
      }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    } else {
      n = 0;                                   // 太长就丢掉重来
    }
  }
}
