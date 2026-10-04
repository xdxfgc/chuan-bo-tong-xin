/* =====================================================================
   lora_link.cpp   LoRa 收发实现
   ---------------------------------------------------------------------
   发送用「非阻塞发射 + 轮询 TxDone」，接收用轮询 IRQ 寄存器，
   所以 DIO0 不用接线。
   ===================================================================== */

#include "lora_link.h"

static bool     s_ready   = false;

/* ---------------- 重传识别 ----------------
   以前只记「上一包的序号」，多只信标交替发时会被误判。
   现在按发送者分开记：来一只新的就占一格，满了就循环覆盖最老的一格。
   ID 0（老格式帧）也按一只“发送者”处理。                              */

#define MAX_TRACKED_SENDERS 8

struct LastSeq {
  uint8_t  id;
  uint32_t seq;
  bool     used;
};

static LastSeq s_lastSeqs[MAX_TRACKED_SENDERS];
static uint8_t s_nextSlot = 0;

// 返回 true 表示「这只发送者、这个序号」刚出现过（重传包）
static bool seenBefore(uint8_t id, uint32_t seq) {
  for (int i = 0; i < MAX_TRACKED_SENDERS; i++) {
    if (s_lastSeqs[i].used && s_lastSeqs[i].id == id) {
      bool dup = (s_lastSeqs[i].seq == seq);
      s_lastSeqs[i].seq = seq;
      return dup;
    }
  }
  for (int i = 0; i < MAX_TRACKED_SENDERS; i++) {
    if (!s_lastSeqs[i].used) {
      s_lastSeqs[i].used = true;
      s_lastSeqs[i].id   = id;
      s_lastSeqs[i].seq  = seq;
      return false;
    }
  }
  s_lastSeqs[s_nextSlot].used = true;
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

/* ---------------- 发一包 ---------------- */

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
   新格式：M,<信标ID>,<序号>,P,<定位有效>,<纬度>,<经度>
   老格式：M,<序号>,P,<定位有效>,<纬度>,<经度>

   怎么区分：老格式的第 3 个字段直接就是 "P"，新格式第 3 个字段是序号。
   所以先切出第 2、3 个字段，看第 3 个字段开头是不是 "P" 就知道了。
   ——不能用“第 2 个字段是不是数字”来判断，因为两种格式那里都是数字。   */

static bool parsePacketString(const String& s, TargetPacket* out) {
  out->beaconId = BEACON_ID_NONE;

  if (!s.startsWith("M,")) return false;

  int p1 = s.indexOf(',');
  if (p1 < 0) return false;
  int p2 = s.indexOf(',', p1 + 1);
  if (p2 < 0) return false;

  String field2 = s.substring(p1 + 1, p2);   // 老格式=序号   新格式=信标ID
  String rest   = s.substring(p2 + 1);       // 老格式="P,..." 新格式="<序号>,P,..."

  String payload;
  if (rest.startsWith("P,")) {               // 老格式
    out->seq = (uint32_t)strtoul(field2.c_str(), nullptr, 10);
    payload  = rest;
  } else {                                   // 新格式：再切一个字段才是序号
    int p3 = rest.indexOf(',');
    if (p3 < 0) return false;
    out->beaconId = (uint8_t)strtoul(field2.c_str(), nullptr, 10);
    out->seq      = (uint32_t)strtoul(rest.substring(0, p3).c_str(), nullptr, 10);
    payload       = rest.substring(p3 + 1);
  }
  payload.trim();

  int    fix = 1;
  double la = 0, lo = 0;
  if (sscanf(payload.c_str(), "P,%d,%lf,%lf", &fix, &la, &lo) != 3) {
    fix = 1;                                 // 兼容不带 P 标签的 "纬度,经度"
    if (sscanf(payload.c_str(), "%lf,%lf", &la, &lo) != 2) return false;
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
     在 ESP32 上那会把总线开到默认脚（SCK=18/MISO=19/MOSI=23/SS=5），
     模块接在别的脚上就永远读不到版本号。先自己开好总线，库那次调用就变成空操作。 */
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
  LoRa.enableCrc();                          // 两端都要开，坏包直接丢
  Serial.printf("[LoRa] 就绪 %.0fMHz SF%d BW%.0fkHz SYNC=0x%02X\n",
                RF_FREQ / 1e6, RF_SF, RF_BW / 1e3, RF_SYNC);
  s_ready = true;
  return true;
}

bool loraIsReady() { return s_ready; }

bool loraPoll(TargetPacket* out) {
  if (!s_ready || out == nullptr) return false;

  int n = LoRa.parsePacket();
  if (n <= 0) return false;

  String s;
  s.reserve(n + 1);
  while (LoRa.available()) s += (char)LoRa.read();

  TargetPacket pkt;
  pkt.rssi = LoRa.packetRssi();
  pkt.snr  = LoRa.packetSnr();

  if (!parsePacketString(s, &pkt)) {
    /* S 帧（别的船或岸基的广播）、A 帧（别处发的应答）本来就不是给本板收的，
       静默丢掉；只有 M 帧是本板该收的，那种解析不了才值得报出来。 */
    if (s.startsWith("M,")) Serial.printf("[LoRa] 收到无法解析的包：%s\n", s.c_str());
    return false;
  }

  pkt.duplicate = seenBefore(pkt.beaconId, pkt.seq);

  *out = pkt;
  return true;
}

/* 应答帧：A,<船ID>,<序号>,C,<本方定位有效>,<纬度>,<经度>
   带上船 ID 之后，多只信标/多条船同时工作时信标端才分得清。        */
void loraSendAck(uint32_t seq, bool centerValid, double lat, double lon) {
  if (!s_ready) return;
  char tx[64];
  if (centerValid) {
    snprintf(tx, sizeof(tx), "A,%u,%lu,C,1,%.6f,%.6f",
             (unsigned)DEV_ID, (unsigned long)seq, lat, lon);
  } else {
    snprintf(tx, sizeof(tx), "A,%u,%lu,C,0,0,0",
             (unsigned)DEV_ID, (unsigned long)seq);
  }
  if (!txPacket(tx)) Serial.println("[LoRa] ACK 发送失败：800ms 内没有 TxDone");
}

/* 发一帧原文。定期广播（S 帧）用它，省得每个模块都去碰射频细节。 */
bool loraSendText(const char* s) {
  if (!s_ready || s == nullptr) return false;
  return txPacket(s);
}

int loraChannelRssi() {
  if (!s_ready) return 0;
  return LoRa.rssi();
}
