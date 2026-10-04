/* =====================================================================
   lora_link.cpp   LoRa 收发实现
   ---------------------------------------------------------------------
   发送用「非阻塞发射 + 轮询 TxDone」，接收用轮询 IRQ 寄存器，
   所以 DIO0 不用接线。
   ===================================================================== */

#include "lora_link.h"

static bool     s_ready   = false;
static uint32_t s_lastSeq = 0;
static bool     s_hasSeq  = false;

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

/* ---------------- 解析 "M,<序号>,<载荷>" ---------------- */

static bool parsePacketString(const String& s, TargetPacket* out) {
  if (!s.startsWith("M,")) return false;
  int p1 = s.indexOf(',');
  int p2 = s.indexOf(',', p1 + 1);
  if (p1 < 0 || p2 < 0) return false;

  out->seq = (uint32_t)strtoul(s.substring(p1 + 1, p2).c_str(), nullptr, 10);

  String payload = s.substring(p2 + 1);
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
    Serial.printf("[LoRa] 收到无法解析的包：%s\n", s.c_str());
    return false;
  }

  pkt.duplicate = (s_hasSeq && pkt.seq == s_lastSeq);
  s_lastSeq = pkt.seq;
  s_hasSeq  = true;

  *out = pkt;
  return true;
}

void loraSendAck(uint32_t seq, bool centerValid, double lat, double lon) {
  if (!s_ready) return;
  char tx[64];
  if (centerValid) {
    snprintf(tx, sizeof(tx), "A,%lu,C,1,%.6f,%.6f", (unsigned long)seq, lat, lon);
  } else {
    snprintf(tx, sizeof(tx), "A,%lu,C,0,0,0", (unsigned long)seq);
  }
  if (!txPacket(tx)) Serial.println("[LoRa] ACK 发送失败：800ms 内没有 TxDone");
}

int loraChannelRssi() {
  if (!s_ready) return 0;
  return LoRa.rssi();
}
