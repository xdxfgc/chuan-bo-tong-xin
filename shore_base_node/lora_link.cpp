/* =====================================================================
   lora_link.cpp   岸基节点 LoRa 收发实现
   ---------------------------------------------------------------------
   发送用「非阻塞发射 + 轮询 TxDone」，接收用轮询 IRQ 寄存器，
   所以 DIO0 不用接线。和船端 lora_link.cpp 的收发底层一致。
   ===================================================================== */

#include "lora_link.h"

static bool s_ready = false;

/* ---------------- 重传识别 ----------------
   按「发送者」分开记：信标不只一只、还有船端，共用一个"上一包序号"
   会互相误判成重传。每个发送者占一格，满了就循环覆盖最老的一格。 */
#define MAX_TRACKED_SENDERS 8

struct LastSeq {
  int      kind;
  int      id;
  uint32_t seq;
  bool     used;
};

static LastSeq s_lastSeqs[MAX_TRACKED_SENDERS];
static uint8_t s_nextSlot = 0;

static bool seenBefore(int kind, int id, uint32_t seq) {
  for (int i = 0; i < MAX_TRACKED_SENDERS; i++) {
    if (s_lastSeqs[i].used && s_lastSeqs[i].kind == kind && s_lastSeqs[i].id == id) {
      bool dup = (s_lastSeqs[i].seq == seq);
      s_lastSeqs[i].seq = seq;
      return dup;
    }
  }
  for (int i = 0; i < MAX_TRACKED_SENDERS; i++) {
    if (!s_lastSeqs[i].used) {
      s_lastSeqs[i].used = true;
      s_lastSeqs[i].kind = kind;
      s_lastSeqs[i].id   = id;
      s_lastSeqs[i].seq  = seq;
      return false;
    }
  }
  s_lastSeqs[s_nextSlot].used = true;
  s_lastSeqs[s_nextSlot].kind = kind;
  s_lastSeqs[s_nextSlot].id   = id;
  s_lastSeqs[s_nextSlot].seq  = seq;
  s_nextSlot = (uint8_t)((s_nextSlot + 1) % MAX_TRACKED_SENDERS);
  return false;
}

/* ---------------- 直接读写 SX127x 中断标志寄存器 ---------------- */

static uint8_t irqFlags() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_NSS_PIN, LOW);
  SPI.transfer(0x12);                        // 读 REG_IRQ_FLAGS
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(LORA_NSS_PIN, HIGH);
  SPI.endTransaction();
  return v;
}

static void clearTxDone() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_NSS_PIN, LOW);
  SPI.transfer(0x92);                        // 写 REG_IRQ_FLAGS
  SPI.transfer(0x08);                        // 清 TxDone
  digitalWrite(LORA_NSS_PIN, HIGH);
  SPI.endTransaction();
}

static bool txPacket(const char* s) {
  LoRa.beginPacket();
  LoRa.print(s);
  LoRa.endPacket(true);                      // true = 非阻塞
  unsigned long t0 = millis();
  while (!(irqFlags() & 0x08) && millis() - t0 < 800) delay(1);
  bool ok = (irqFlags() & 0x08);
  clearTxDone();
  LoRa.idle();
  return ok;
}

/* ---------------- 解析入帧 ----------------
   新格式：M/R/S/A,<发送者ID>,<序号>,<载荷>
   老格式：M/S/A,<序号>,<载荷>              （没有发送者ID）

   怎么区分：**第 3 个字段**直接以 P 或 C 开头就是老格式（载荷提前了）。
   不能用"第 2 个字段是不是数字"判断 —— 两种格式那里都是数字。       */

static bool parsePacketString(const String& s, LoraPacket* out) {
  if (s.length() < 3 || s[1] != ',') return false;

  char k = s[0];
  if      (k == 'M') out->kind = LK_BEACON;
  else if (k == 'S') out->kind = LK_VESSEL;
  else if (k == 'R') out->kind = LK_SHORE;
  else if (k == 'A') out->kind = LK_ACK;
  else return false;                         // 不认识的帧，丢掉

  int p1 = s.indexOf(',');                   // 前缀后第一个逗号
  if (p1 < 0) return false;
  int p2 = s.indexOf(',', p1 + 1);
  if (p2 < 0) return false;

  String field2 = s.substring(p1 + 1, p2);   // 老格式=序号   新格式=发送者ID
  String rest   = s.substring(p2 + 1);       // 老格式="P,…"  新格式="<序号>,P,…"

  String payload;
  if (rest.startsWith("P,") || rest.startsWith("C,") ||
      rest.startsWith("p,") || rest.startsWith("c,")) {
    out->srcId = 0;                          // 老格式，没带编号
    out->seq   = (uint32_t)strtoul(field2.c_str(), nullptr, 10);
    payload    = rest;
  } else {
    int p3 = rest.indexOf(',');
    if (p3 < 0) return false;
    out->srcId = (int)strtol(field2.c_str(), nullptr, 10);   // 新格式
    out->seq   = (uint32_t)strtoul(rest.substring(0, p3).c_str(), nullptr, 10);
    payload    = rest.substring(p3 + 1);
  }
  payload.trim();

  /* 前四个字段三种帧都有：<标签>, <定位有效>, <纬度>, <经度>
     第 5 个字段起，**信标帧和船端帧的意思不一样**，所以要按 kind 分开读：
         信标帧：,<水感 0/1>
         船端帧：,<对地速度节>,<航向度>,<卫星数>
     分开读还有个好处：船端的速度是小数（0.00），用 %d 读会被截断，
     混在一个 sscanf 里会读错。                                      */
  int    fix = 0;
  double la = 0, lo = 0;

  if (out->kind == LK_BEACON) {
    int water = 0;
    int got = sscanf(payload.c_str(), "%*[^,],%d,%lf,%lf,%d", &fix, &la, &lo, &water);
    if (got < 3) {
      fix = 1;
      if (sscanf(payload.c_str(), "%lf,%lf", &la, &lo) < 2) return false;
    } else if (got >= 4) {
      out->hasWater = true;
      out->waterOn  = (water != 0);
    }
  } else {
    float sog = 0.0f, cog = 0.0f;
    int   sats = 0;
    int got = sscanf(payload.c_str(), "%*[^,],%d,%lf,%lf,%f,%f,%d",
                     &fix, &la, &lo, &sog, &cog, &sats);
    if (got < 3) {
      fix = 1;
      if (sscanf(payload.c_str(), "%lf,%lf", &la, &lo) < 2) return false;
    } else if (got >= 6) {
      out->hasExtra = true;
      out->sogKnots = sog;
      out->cogDeg   = cog;
      out->sats     = sats;
    }
  }

  // 越界或 (0,0) 一律当作无效
  if (fabs(la) > 90.0 || fabs(lo) > 180.0 || (fabs(la) < 1e-9 && fabs(lo) < 1e-9)) {
    out->valid = false;
    out->lat = out->lon = 0.0;
    return true;
  }

  out->lat   = la;
  out->lon   = lo;
  out->valid = (fix != 0);
  return true;
}

/* ---------------- 对外接口 ---------------- */

bool loraBegin() {
  /* 这一行不能省：LoRa 库内部只调用不带参数的 SPI.begin()，
     在 ESP32 上那会把总线开到默认脚，模块接在别的脚上就永远读不到版本号。 */
  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);

  LoRa.setPins(LORA_NSS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  if (!LoRa.begin(RF_FREQ)) {
    Serial.println("[LoRa] 初始化失败：读不到 SX1278。检查 3V3/GND 和 "
                   "SCK=14 MISO=19 MOSI=23 NSS=13 RST=27");
    s_ready = false;
    return false;
  }
  LoRa.setSpreadingFactor(RF_SF);
  LoRa.setSignalBandwidth(RF_BW);
  LoRa.setSyncWord(RF_SYNC);
  LoRa.setTxPower(17);
  LoRa.enableCrc();                          // 三端都要开，坏包直接丢
  Serial.printf("[LoRa] 就绪 %.0fMHz SF%d BW%.0fkHz SYNC=0x%02X  本机编号 %d\n",
                RF_FREQ / 1e6, RF_SF, RF_BW / 1e3, RF_SYNC, DEV_ID);
  s_ready = true;
  return true;
}

bool loraIsReady() { return s_ready; }

bool loraPoll(LoraPacket* out) {
  if (!s_ready || out == nullptr) return false;

  int n = LoRa.parsePacket();
  if (n <= 0) return false;

  String s;
  s.reserve(n + 1);
  while (LoRa.available()) s += (char)LoRa.read();

  LoraPacket pkt;
  pkt.rssi = LoRa.packetRssi();
  pkt.snr  = LoRa.packetSnr();

  if (!parsePacketString(s, &pkt)) {
    /* 别的岸基发的 R 帧、或者不认识的帧，本来就不是给本节点收的，
       静默丢掉；只有 M/S/A 是本节点该收的，解析不了才值得报出来。 */
    if (s.startsWith("M,") || s.startsWith("S,") || s.startsWith("A,"))
      Serial.printf("[LoRa] 收到无法解析的包：%s\n", s.c_str());
    return false;
  }

  pkt.duplicate = seenBefore((int)pkt.kind, pkt.srcId, pkt.seq);

  *out = pkt;
  return true;
}

bool loraSendText(const char* s) {
  if (!s_ready || s == nullptr) return false;
  if (!txPacket(s)) {
    Serial.printf("[LoRa] 发送失败：%s\n", s);
    return false;
  }
  return true;
}

/* 应答帧：A,<自己ID>,<序号>,C,<自己定位有效>,<纬度>,<经度>
   带上自己的编号，信标/船端才知道是谁应的。                       */
void loraSendAck(int seq, bool myValid, double lat, double lon) {
  char tx[64];
  if (myValid) {
    snprintf(tx, sizeof(tx), "A,%d,%d,C,1,%.6f,%.6f", DEV_ID, seq, lat, lon);
  } else {
    snprintf(tx, sizeof(tx), "A,%d,%d,C,0,0,0", DEV_ID, seq);
  }
  loraSendText(tx);
}

int loraChannelRssi() {
  if (!s_ready) return 0;
  return LoRa.rssi();
}
