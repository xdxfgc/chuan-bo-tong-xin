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

/* ==================== I2C 小工具（裸 Wire 操作） ====================
   为什么需要这些：VL53L1X 的 I2C 地址写进芯片就一直在，**不随 ESP32 复位恢复**。
   所以我们把船头那只改成 0x2A 之后，只要 ESP32 重启（按 EN、重新烧录、看门狗复位）
   而激光模块没断电，它就还停在 0x2A —— 下次开机代码去 0x29 找它，必然找不到，
   日志就会出现「船头初始化失败、右舷正常」。
   下面的函数用裸 I2C 直接跟 0x2A 说话，把它请回 0x29。              */

// 探一下某个地址有没有器件应答（只发地址，不读写寄存器，安全）
static bool busProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// 把某只 VL53L1X 的地址改成 to（写寄存器 0x0001 即可，与库的 setAddress 等价）
static bool busSetAddress(uint8_t from, uint8_t to) {
  Wire.beginTransmission(from);
  Wire.write(0x00);              // 寄存器地址高字节
  Wire.write(0x01);              // 寄存器地址低字节 → 0x0001 = 器件地址寄存器
  Wire.write(to & 0x7F);
  return Wire.endTransmission() == 0;
}

// 把总线上真实存在的地址打印出来（初始化失败时用，一眼看出谁在谁不在）
static void busPrintDevices() {
  Serial.print("当前 I2C 总线上的器件：");
  uint8_t n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (busProbe(a)) { Serial.printf("0x%02X ", a); n++; }
  }
  if (n == 0) Serial.print("一个都没有（供电/共地/SDA/SCL 要查）");
  Serial.println();
}

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

  /* 船头那只没有 XSHUT 线，ESP32 复位时它不跟着复位：
     上次被改成 0x2A 的地址会一直留着。这里先探一下，把它请回 0x29，
     否则下面的船头初始化必然失败（就是日志里那个现象）。 */
  if (!busProbe(0x29) && busProbe(TOF_ADDR_BOW)) {
    Serial.printf("检测到船头那只还停在 0x%02X（上次运行残留的地址，芯片没断电不会自己复位），"
                  "正在改回 0x29…\n", TOF_ADDR_BOW);
    busSetAddress(TOF_ADDR_BOW, 0x29);
    delay(10);
  }
#endif

  /* ---- 船头那只 ---- */
  tofBow.setTimeout(500);
  if (!tofBow.init()) {
    bowReady = false;
    Serial.println("激光测距 VL53L1X（船头）初始化失败，按下面扫到的地址对号入座：");
    Serial.println("  只有 0x3C / 0x68（没有激光）→ 这只根本没上总线：VIN 要 3.3V、GND 共地、"
                   "SDA=GPIO21、SCL=GPIO22 别接反");
    Serial.println("  有 0x2A 却没能自动改回来 → 模块供电不稳 / 杜邦线接触不良");
    busPrintDevices();
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
    busPrintDevices();
  } else {
    tofConfig(tofSide);
    sideReady = true;
    if (bowReady)
      Serial.printf("激光测距（右舷）已启动：地址 0x%02X（船头那只在 0x%02X，两路都在）\n",
                    tofSide.getAddress(), tofBow.getAddress());
    else
      Serial.printf("激光测距（右舷）已启动：地址 0x%02X（船头那只这次没起来）\n",
                    tofSide.getAddress());
  }
#endif
}

void tofUpdate() {
  if (millis() - tofLastReadMs < TOF_READ_MS) return;
  tofLastReadMs = millis();

  if (bowReady) {
    /* 读之前先问一句"这次测好了吗"。非阻塞读遇到"还没测好"会直接返回 0，
       而 ranging_data.range_status 还留着上一次的值（通常正好是"正常"），
       于是 0 毫米会被当成一次真读数，串口上就出现「0 mm (0.00 m)」。
       VL53L1X 的盲区是 4 厘米，0 毫米一定是无效值，这里两道都挡上。 */
    if (!tofBow.dataReady()) {
      bowValid = false;
    } else {
      uint16_t d = tofBow.read(false);
      if (tofBow.timeoutOccurred() || d == 0) {
        bowValid = false;
      } else if (tofBow.ranging_data.range_status == VL53L1X::RangeValid) {
        bowMm    = d;
        bowValid = true;
      } else {
        bowValid = false;
      }
    }
  }

#if TOF_SIDE_ENABLE
  if (sideReady) {
    if (!tofSide.dataReady()) {
      sideValid = false;
    } else {
      uint16_t d = tofSide.read(false);
      if (tofSide.timeoutOccurred() || d == 0) {
        sideValid = false;
      } else if (tofSide.ranging_data.range_status == VL53L1X::RangeValid) {
        sideMm    = d;
        sideValid = true;
      } else {
        sideValid = false;
      }
    }
  }
#endif
}

bool     tofIsReady()    { return bowReady; }
bool     tofIsValid()    { return bowValid; }
uint16_t tofDistanceMm() { return bowMm; }

/* 把"测距失败原因"翻译成人话 —— 直接告诉你该往哪个方向查。
   状态码来自 VL53L1X 库的 RangeStatus 枚举。                        */
static const char* tofFailReason(uint8_t st) {
  switch (st) {
    case VL53L1X::RangeValid:                return "正常";
    case VL53L1X::SigmaFail:                 return "信号弱：目标太远、太黑，或环境光太强";
    case VL53L1X::SignalFail:                return "回波太弱：目标太黑/太斜，或镜头脏了";
    case VL53L1X::RangeValidMinRangeClipped: return "贴在最小量程边上（约 4cm）";
    case VL53L1X::OutOfBoundsFail:           return "超出量程：目标太远（>4m）或没有反射面";
    case VL53L1X::HardwareFail:              return "硬件故障：传感器可能坏了";
    case VL53L1X::RangeValidNoWrapCheckFail: return "正常（未做缠绕检查）";
    case VL53L1X::WrapTargetFail:            return "相位缠绕：目标太远";
    case VL53L1X::XtalkSignalFail:           return "串扰：镜头前面有东西挡着";
    case VL53L1X::SynchronizationInt:        return "同步中断（刚启动那一下，忽略即可）";
    case VL53L1X::MinRangeFail:              return "目标太近（小于 4cm）";
    default: {
      static char b[24];
      snprintf(b, sizeof(b), "未知状态 %u", (unsigned)st);
      return b;
    }
  }
}

String tofText() {
  if (!bowReady) return "模块未连接";
  if (!bowValid) {
    char b[80];
    snprintf(b, sizeof(b), "无效（%s）", tofFailReason(tofBow.ranging_data.range_status));
    return String(b);
  }
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
  if (!sideValid) {
    char b[80];
    snprintf(b, sizeof(b), "无效（%s）", tofFailReason(tofSide.ranging_data.range_status));
    return String(b);
  }
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

/* ==================== 自检（串口输入 tof 触发） ====================
   一路读 5 次，每次把「距离 / 状态 / 回波强度 / 环境光」全打出来。
   为什么要看后面两个数：它们是芯片自己量到的光强，
     · 超时             → 芯片根本没测完一次，不是"看不到东西"，是它没在干活
     · 回波强度 ≈ 0     → 光发出去没回来：镜头被挡、保护膜没撕、对着水面或空处
     · 环境光很大       → 太亮，挪开阳光或射灯
   "测不到"到底怪谁，看这两个数就分得清了。                                */
static void tofSelfTestOne(VL53L1X& t, const char* name, bool ready) {
  Serial.printf("【%s】", name);
  if (!ready) {
    Serial.println("没起来：芯片不应答。查 VIN=3.3V、GND 共地、SDA=GPIO21、SCL=GPIO22"
                   "（右舷那只还要查 XSHUT=GPIO32）");
    return;
  }

  Serial.printf("地址 0x%02X，读 5 次：\n", t.getAddress());
  for (int i = 1; i <= 5; i++) {
    t.setTimeout(150);
    uint16_t d = t.read(true);                       // 阻塞读，最多等 150ms
    if (t.timeoutOccurred())
      Serial.printf("  %d) 超时：150 毫秒都没测完一次 —— 芯片没在正常测距\n", i);
    else
      Serial.printf("  %d) %u mm  状态：%s  回波 %.2f MCPS  环境光 %.2f MCPS\n",
                    i, (unsigned)d, tofFailReason((uint8_t)t.ranging_data.range_status),
                    t.ranging_data.peak_signal_count_rate_MCPS,
                    t.ranging_data.ambient_count_rate_MCPS);
    delay(60);
  }
}

void tofSelfTest() {
  Serial.println("=========== 激光测距自检 ===========");
  tofSelfTestOne(tofBow, "船头", bowReady);
#if TOF_SIDE_ENABLE
  tofSelfTestOne(tofSide, "右舷", sideReady);
#else
  Serial.println("【右舷】未启用（config.h 里 TOF_SIDE_ENABLE = 0）");
#endif
  Serial.println("====================================");
}
