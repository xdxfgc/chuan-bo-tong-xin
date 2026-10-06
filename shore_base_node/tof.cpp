/* =====================================================================
   tof.cpp   岸基节点激光测距实现
   ---------------------------------------------------------------------
   初始化失败只提示、不阻塞：tofReady 保持 false，后续读取直接跳过，
   主程序其余部分（LoRa / 网页 / 语音 / OLED）照常运行。
   ===================================================================== */

#include "tof.h"

static VL53L1X tof;
static bool     tofReady = false;
static bool     tofValid = false;
static uint16_t tofMm    = 0;
static unsigned long tofLastReadMs = 0;

void tofBegin() {
  Wire.begin(TOF_SDA_PIN, TOF_SCL_PIN);
  Wire.setClock(400000);
  tof.setTimeout(500);

  if (!tof.init()) {
    tofReady = false;
    Serial.println("激光测距 VL53L1X 初始化失败：检查 VIN 是否接 3.3V、GND 是否共地、SDA/SCL 是否接对。");
    return;
  }

  tof.setDistanceMode(TOF_LONG_RANGE ? VL53L1X::Long : VL53L1X::Short);
  tof.setMeasurementTimingBudget(50000);
  tof.startContinuous(50);
  tofReady = true;
  Serial.printf("激光测距 VL53L1X 已启动：SDA=GPIO%d  SCL=GPIO%d  %s模式\n",
                TOF_SDA_PIN, TOF_SCL_PIN, TOF_LONG_RANGE ? "远距" : "短距");
}

void tofUpdate() {
  if (!tofReady) return;
  if (millis() - tofLastReadMs < TOF_READ_MS) return;
  tofLastReadMs = millis();

  uint16_t d = tof.read(false);
  if (tof.timeoutOccurred()) {
    tofValid = false;
    return;
  }
  if (tof.ranging_data.range_status == VL53L1X::RangeValid) {
    tofMm    = d;
    tofValid = true;
  } else {
    tofValid = false;
  }
}

bool     tofIsReady()    { return tofReady; }
bool     tofIsValid()    { return tofValid; }
uint16_t tofDistanceMm() { return tofMm; }

String tofText() {
  if (!tofReady) return "模块未连接";
  if (!tofValid) return "无效（超出量程或信号弱）";
  char b[32];
  snprintf(b, sizeof(b), "%u mm (%.2f m)", (unsigned)tofMm, tofMm / 1000.0);
  return String(b);
}

void tofPrintReport() {
  Serial.printf("岸侧激光 : %s\n", tofText().c_str());
}