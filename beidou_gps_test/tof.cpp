/* =====================================================================
   tof.cpp   激光测距实现（船头 + 右舷 两路）
   ---------------------------------------------------------------------
   两只 VL53L1X 出厂地址都是 0x29，必须靠 XSHUT 把地址分开才能挂一条总线：
     1) 先把右舷那只按住（XSHUT 拉低 = 复位）
     2) 船头那只正常初始化，拿到默认地址 0x29
     3) 把船头那只改到 0x2A，让出 0x29
     4) 放开右舷那只，它回到 0x29，再初始化
   这四步必须在任何 I2C 操作之前做完，顺序也不能换。
   ===================================================================== */

#include "tof.h"

static VL53L1X  tofBow;                        // 船头那只
#if TOF_SIDE_ENABLE
static VL53L1X  tofSide;                       // 右舷那只
#endif

static bool     bowReady  = false;
static bool     bowValid  = false;
static uint16_t bowMm     = 0;

static bool     sideReady = false;
static bool     sideValid = false;
static uint16_t sideMm    = 0;

static unsigned long tofLastReadMs = 0;

// 配置一只传感器：测距模式、时间预算、开始连续测量
static void tofConfig(VL53L1X& t) {
  t.setDistanceMode(TOF_LONG_RANGE ? VL53L1X::Long : VL53L1X::Short);
  t.setMeasurementTimingBudget(50000);
  t.startContinuous(50);
}

void tofBegin() {
  Wire.begin(TOF_SDA_PIN, TOF_SCL_PIN);
  Wire.setClock(400000);

#if TOF_SIDE_ENABLE
  pinMode(TOF2_XSHUT_PIN, OUTPUT);
  digitalWrite(TOF2_XSHUT_PIN, LOW);      // 右舷那只先按住，别来抢 0x29
  delay(20);
#endif

  /* ---- 船头那只 ---- */
  tofBow.setTimeout(500);
  if (!tofBow.init()) {
    bowReady = false;
    Serial.println("激光测距 VL53L1X（船头）初始化失败：检查 VIN 是否接 3.3V、"
                   "GND 是否共地、SDA/SCL 是否接对。");
  } else {
#if TOF_SIDE_ENABLE
    tofBow.setAddress(TOF_ADDR_BOW);      // 让出 0x29 给右舷那只
    delay(10);
#endif
    tofConfig(tofBow);
    bowReady = true;
    Serial.printf("激光测距（船头）已启动：SDA=GPIO%d SCL=GPIO%d 地址 0x%02X  %s模式\n",
                  TOF_SDA_PIN, TOF_SCL_PIN, tofBow.getAddress(),
                  TOF_LONG_RANGE ? "远距" : "短距");
  }

#if TOF_SIDE_ENABLE
  /* ---- 右舷那只：放开复位，它回到默认 0x29 ---- */
  digitalWrite(TOF2_XSHUT_PIN, HIGH);
  delay(20);
  tofSide.setTimeout(500);
  if (!tofSide.init()) {
    sideReady = false;
    Serial.println("激光测距 VL53L1X（右舷）初始化失败：先查 XSHUT 有没有接 GPIO32、"
                   "SDA/SCL 有没有和船头那只并联、VIN 是不是 3.3V。");
  } else {
    tofConfig(tofSide);
    sideReady = true;
    Serial.printf("激光测距（右舷）已启动：地址 0x%02X（和船头那只 0x%02X 分开）\n",
                  tofSide.getAddress(), tofBow.getAddress());
  }
#endif
}

void tofUpdate() {
  if (millis() - tofLastReadMs < TOF_READ_MS) return;
  tofLastReadMs = millis();

  if (bowReady) {
    uint16_t d = tofBow.read(false);
    if (tofBow.timeoutOccurred()) {
      bowValid = false;
    } else if (tofBow.ranging_data.range_status == VL53L1X::RangeValid) {
      bowMm    = d;
      bowValid = true;
    } else {
      bowValid = false;
    }
  }

#if TOF_SIDE_ENABLE
  if (sideReady) {
    uint16_t d = tofSide.read(false);
    if (tofSide.timeoutOccurred()) {
      sideValid = false;
    } else if (tofSide.ranging_data.range_status == VL53L1X::RangeValid) {
      sideMm    = d;
      sideValid = true;
    } else {
      sideValid = false;
    }
  }
#endif
}

bool     tofIsReady()    { return bowReady; }
bool     tofIsValid()    { return bowValid; }
uint16_t tofDistanceMm() { return bowMm; }

String tofText() {
  if (!bowReady) return "模块未连接";
  if (!bowValid) return "无效（超出量程或信号弱）";
  char b[32];
  snprintf(b, sizeof(b), "%u mm (%.2f m)", (unsigned)bowMm, bowMm / 1000.0);
  return String(b);
}

#if TOF_SIDE_ENABLE
bool     tofSideIsReady()    { return sideReady; }
bool     tofSideIsValid()    { return sideValid; }
uint16_t tofSideDistanceMm() { return sideMm; }

String tofSideText() {
  if (!sideReady) return "模块未连接";
  if (!sideValid) return "无效（超出量程或信号弱）";
  char b[32];
  snprintf(b, sizeof(b), "%u mm (%.2f m)", (unsigned)sideMm, sideMm / 1000.0);
  return String(b);
}
#else
bool     tofSideIsReady()    { return false; }
bool     tofSideIsValid()    { return false; }
uint16_t tofSideDistanceMm() { return 0; }
String   tofSideText()       { return "未安装"; }
#endif

void tofPrintReport() {
  Serial.printf("激光前方 : %s\n", tofText().c_str());
#if TOF_SIDE_ENABLE
  Serial.printf("激光右舷 : %s\n", tofSideText().c_str());
#endif
}
