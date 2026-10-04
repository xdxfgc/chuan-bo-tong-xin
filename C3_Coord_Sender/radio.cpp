/* =====================================================================
   radio.cpp   LoRa 收发实现
   ---------------------------------------------------------------------
   直接读写 SX127x 的中断标志寄存器来判断 TxDone，
   所以不需要把 DIO0 接到单片机上。
   ===================================================================== */

#include "radio.h"

static bool  s_ready = false;
static int   s_rssi  = 0;
static float s_snr   = 0.0f;

/* ---------------- 中断标志寄存器 ---------------- */

static uint8_t irqFlags() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(NSS_PIN, LOW);
  SPI.transfer(0x12);                  // 读 REG_IRQ_FLAGS
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(NSS_PIN, HIGH);
  SPI.endTransaction();
  return v;
}

static void clearTxDone() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(NSS_PIN, LOW);
  SPI.transfer(0x92);                  // 写 REG_IRQ_FLAGS
  SPI.transfer(0x08);                  // 清 TxDone
  digitalWrite(NSS_PIN, HIGH);
  SPI.endTransaction();
}

/* ---------------- 对外接口 ---------------- */

bool radioBegin() {
  /* 这一行不能省：LoRa 库内部只调用不带参数的 SPI.begin()，
     在 ESP32 上那会把总线开到默认脚，模块接在别的脚上就永远读不到版本号。
     先按实际接线把总线开好，库后面那次调用就变成空操作。 */
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, NSS_PIN);

  LoRa.setPins(NSS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(RF_FREQ)) {
    DBG.println("LoRa 初始化失败! 读不到 SX1278 —— 检查模块 3V3/GND 与 "
                "SCK=6、MISO=2、MOSI=7、NSS=10、RST=8");
    s_ready = false;
    return false;
  }
  LoRa.setSpreadingFactor(RF_SF);
  LoRa.setSignalBandwidth(RF_BW);
  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setTxPower(17);
  LoRa.enableCrc();                    // 与船端一致：坏包直接丢

  DBG.printf("【坐标发送端】就绪  %.0fMHz SF%d BW%.0fkHz SYNC=0x%02X\n",
             RF_FREQ / 1e6, RF_SF, RF_BW / 1e3, SYNC_WORD);
  s_ready = true;
  return true;
}

bool radioIsReady() { return s_ready; }

bool radioSend(const char* s) {
  if (!s_ready) return false;

  LoRa.beginPacket();
  LoRa.print(s);
  LoRa.endPacket(true);                // true = 非阻塞

  unsigned long t0 = millis();
  while (!(irqFlags() & 0x08) && millis() - t0 < TX_TIMEOUT_MS) delay(1);
  bool ok = (irqFlags() & 0x08);
  clearTxDone();
  LoRa.idle();
  return ok;
}

bool radioReceive(String& out, uint32_t timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    int n = LoRa.parsePacket();
    if (n) {
      s_rssi = LoRa.packetRssi();
      s_snr  = LoRa.packetSnr();
      out = "";
      out.reserve(n);
      while (LoRa.available()) out += (char)LoRa.read();
      return true;
    }
    delay(1);
  }
  return false;
}

int   radioLastRssi() { return s_rssi; }
float radioLastSnr()  { return s_snr; }
