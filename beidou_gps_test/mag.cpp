/* ============================================================
 *  mag.cpp —— GY-282（HMC5983）三轴磁力计驱动（实现）
 *
 *  为什么自己写驱动：HMC5983 和 HMC5883L 寄存器完全兼容，一共十来个
 *  寄存器，写清楚了几十行就能用，现场不用装库、不挑版本。
 *
 *  三个必须知道的坑，代码里都处理了：
 *    1) 数据寄存器顺序是 X、Z、Y（不是 X、Y、Z），读错就"乱转"
 *    2) 连续测量模式下 DRDY 位不可靠，所以按输出速率定时读
 *    3) 地磁场只有几十 µT，而芯片自身零偏就有几百 µT：
 *       不标定的话指北能偏几十度 —— 所以有 cal auto 标定
 *
 *  关于片上温度（MAG_TEMP_ENABLE）：
 *    HMC5983 比 HMC5883L 多一个温度传感器，寄存器 0x31/0x32。
 *    它的换算关系在不同批次资料里说法不一，本代码按
 *    raw/8 + 偏置 处理，偏置用 MAG_TEMP_OFFSET_C 现场校准。
 *    默认关闭；要用先拿室温对一下，不对就调偏置。
 * ============================================================ */

#include "mag.h"
#include "config.h"
#include <Wire.h>
#include <Preferences.h>

/* ---------- 常量表（数据手册 Table 1 / Table 2） ---------- */
/* 各量程档位的灵敏度，单位 LSB/Gauss */
static const float    LSB_PER_GAUSS[8] = {1370.0f, 1090.0f, 820.0f, 660.0f,
                                           440.0f,  390.0f, 330.0f, 230.0f};
/* 各输出速率档位两次采样之间的间隔（ms） */
static const uint16_t RATE_MS[8] = {1333, 667, 333, 133, 67, 33, 13, 5};
static const char*    RATE_NAME[8] = {"0.75Hz", "1.5Hz", "3Hz", "7.5Hz",
                                      "15Hz", "30Hz", "75Hz", "220Hz"};
static const char*    GAIN_NAME[8] = {"+-0.88Ga", "+-1.3Ga", "+-1.9Ga", "+-2.5Ga",
                                      "+-4.0Ga", "+-4.7Ga", "+-5.6Ga", "+-8.1Ga"};

/* HMC5983 寄存器地址 */
#define REG_CRA   0x00      // 配置 A：平均次数 / 输出速率 / 测量模式
#define REG_CRB   0x01      // 配置 B：量程
#define REG_MR    0x02      // 模式：连续 / 单次 / 空闲
#define REG_DATA  0x03      // 数据区起点，连读 6 字节
#define REG_SR    0x09      // 状态
#define REG_IDA   0x0A      // 识别寄存器 A/B/C，正常读回 "H43"
#define REG_TEMP  0x31      // 温度（HMC5983 独有）

/* ==================== 内部状态 ==================== */
static TwoWire*     sWire      = &Wire;
static const char*  sBusName   = "Wire";   // 用哪条 I2C：Wire（与激光共用）或 Wire1
static Preferences  sPrefs;

static bool    sPresent   = false;   // I2C 有应答
static bool    sIdOk      = false;   // ID 是 "H43"
static uint8_t sIdA = 0, sIdB = 0, sIdC = 0;
static uint8_t sProbeFails = 0;      // 连续认不到的次数，攒够 3 次才判定掉线

static int8_t  sAvg  = MAG_AVG;
static int8_t  sRate = MAG_RATE;
static int8_t  sGain = MAG_GAIN;
static int8_t  sMode = MAG_MODE;
static float   sDecl = MAG_DECL_DEG;

/* 标定值：off* 是硬磁中心（µT），sc* 是软磁比例（无量纲） */
static float   sOffX = 0, sOffY = 0, sOffZ = 0;
static float   sScX  = 1, sScY  = 1, sScZ  = 1;
static bool    sCalFromFlash = false;
static uint32_t sCalSavedAtMs = 0;

/* 出发点：记的是"磁北系"里的角度（不含磁偏角），这样以后改磁偏角，
 * 相对航向不会跟着变。对外显示时会再加上磁偏角换算成绝对航向。 */
static float   sZeroMagDeg = 0.0f;
static bool    sZeroSet    = false;

/* 标定值一变，磁北系整体就转了，原来的出发点不再成立：直接清掉并让
 * 调用方在消息里提示一句。返回 true 表示"确实清掉了一个已设的出发点"。 */
static bool zeroInvalidate() {
  if (!sZeroSet) return false;
  sZeroSet    = false;
  sZeroMagDeg = 0.0f;
  sPrefs.remove("zero");
  return true;
}

/* 最近一帧 */
static int16_t sRawX = 0, sRawY = 0, sRawZ = 0;
static float   sX = 0, sY = 0, sZ = 0;          // 平滑后的 µT
static float   sXs = 0, sYs = 0, sZs = 0;       // 平滑器内部状态
static bool    sSmoothReady = false;
static float   sHeading = 0, sHeadingRaw = 0;

static uint32_t sSamples = 0, sErrors = 0;
static unsigned long sLastReadMs = 0;
static unsigned long sLastGoodMs = 0;      // 最近一次成功读到数据的时刻
static uint8_t       sConsecFails = 0;     // 连续失败次数，攒够就复位一次总线
static uint32_t      sRecovers    = 0;     // 总线复位次数
static uint16_t      sErrCode[8]  = {0};   // 错误码分布，见 i2cErr()
static uint8_t       sLastErrCode = 0;
static unsigned long sPrevUpdateMs = 0;    // 上一次进 magUpdate 的时刻
static uint32_t      sGapMaxMs     = 0;    // 两次 magUpdate 之间最长的间隔
static uint32_t      sReadMaxMs    = 0;    // 读一帧（含重试）最长的耗时
static uint8_t       sPullSda      = 2;    // 上电时 SDA 空闲电平（2=没测）
static uint8_t       sPullScl      = 2;    // 上电时 SCL 空闲电平
static uint32_t      sRiseSdaUs    = 0;    // SDA 从低放到高的上升时间（µs）
static uint32_t      sRiseSclUs    = 0;    // SCL 同上
static uint32_t      sBusHz        = I2C_HZ;   // 当前实际用的时钟
static int8_t        sSpeedStep    = 0;        // 降速档：0=最快，最大 3
static uint32_t      sSpeedMoves   = 0;        // 自动降过几次
static uint8_t       sWinTotal     = 0;        // 最近窗口读了几个样本
static uint8_t       sWinFail      = 0;        // 其中失败几个

/* 降速档位：真解决靠外接 4.7k 上拉，代码只能把时钟降到总线上拉能撑住的档位。
 * 上拉越弱，线上从低恢复到高的时间越长，时钟快了就会在"还没到高电平"的时候
 * 就被采样，表现为偶发读失败、数值卡住。 */
static uint32_t speedFor(int step) {
  switch (step) {
    case 0:  return I2C_HZ;
    case 1:  return I2C_HZ / 2;
    case 2:  return I2C_HZ / 5;
    default: return I2C_HZ / 10;
  }
}

/* 按上升时间选初始档位。粗判据来自 I2C 规范：100kHz 要求上升 <1µs、
 * 400kHz 要求 <0.3µs。达不到就往下降档。 */
static int stepForRise(uint32_t riseUs) {
  if (riseUs == 0 || riseUs <= 1) return 0;   // 一放开几乎立刻到高：上拉够硬
  if (riseUs <= 3)                return 1;
  if (riseUs <= 8)                return 2;
  return 3;                                   // 慢得离谱
}
static unsigned long sReadInterval = MAG_READ_MS;
static unsigned long sLastProbeMs = 0;

/* 标定过程 */
static bool   sCalRunning  = false;
static bool   sCalFinished = false;
static String sCalMessage;
static unsigned long sCalEndMs = 0;
static bool   sCalHasSample = false;
static float  sCalMinX, sCalMaxX, sCalMinY, sCalMaxY, sCalMinZ, sCalMaxZ;

static const float PI_F = 3.14159265358979f;

/* ==================== I2C 底层 ==================== */
/* 总线卡死（从机把 SDA 拉住不放）时，手动打 9 个时钟把它放掉，再重开 Wire。
 * 杜邦线接触不良、热插拔、旁边有大电流开关时都会碰上这种情况。 */
static void i2cRecover() {
  sRecovers++;
  sWire->end();
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  for (uint8_t i = 0; i < 9; i++) {          // 9 个时钟足以把卡住的一字节挤完
    pinMode(I2C_SCL, OUTPUT);
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(5);
    pinMode(I2C_SCL, INPUT_PULLUP);
    delayMicroseconds(5);
  }
  pinMode(I2C_SDA, OUTPUT);                  // 补一个 STOP：SCL 高时 SDA 由低变高
  digitalWrite(I2C_SDA, LOW);
  delayMicroseconds(5);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(I2C_SDA, INPUT_PULLUP);
  delayMicroseconds(5);

  sWire->begin(I2C_SDA, I2C_SCL, I2C_HZ);
  sProbeFails = 0;
}

/* 量上升时间：把线拉低一段时间再放开，看多久回到高电平。
 * 实测值 ≈ 上拉电阻 × 总线电容，是判断"上拉够不够硬"最直接的指标，
 * 比只看电平高低有用得多（有上拉但上拉很弱时，电平也是高的）。 */
static uint32_t measureRiseUs(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(50);
  pinMode(pin, INPUT_PULLUP);          // 放开，让上拉把它拉回来
  uint32_t t0 = micros();
  while (digitalRead(pin) == LOW) {
    if (micros() - t0 > 300UL) return 300UL;   // 300µs 还上不来：基本等于没有上拉
  }
  return micros() - t0;
}

/* 记一次 I2C 错误码：1=数据太长 2=地址无应答 3=数据无应答 4=其它 5=总线超时
 * 6=读不到字节。含义差别很大：
 *   2 多半是模块没电/被复位/地址不对；5 是总线被拉死（线松、没上拉）；
 *   3 是通信中途丢了（干扰、电平不匹配）。 */
static void i2cErr(uint8_t code) {
  sLastErrCode = code;
  if (code < 8) sErrCode[code]++;
}

static bool writeReg(uint8_t reg, uint8_t val) {
  sWire->beginTransmission(MAG_ADDR);
  sWire->write(reg);
  sWire->write(val);
  uint8_t rc = sWire->endTransmission();
  if (rc) i2cErr(rc);
  return rc == 0;
}

static bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
  sWire->beginTransmission(MAG_ADDR);
  sWire->write(reg);
  /* 这里刻意发 STOP（endTransmission(true)）再另外发起一次读，不用"重复起始
   * (repeated start)"。HMC5983 的寄存器指针在 STOP 之后会保持，两次事务
   * 拼起来照样能读；而 ESP32 核心 3.x 的 Wire 在"写不发 STOP、紧接着读"
   * 这条路上会偶发拿不到数据（表现就是本文件的错误码 6）。 */
  uint8_t rc = sWire->endTransmission(true);
  if (rc != 0) { i2cErr(rc); return false; }
  delayMicroseconds(100);                    // 给从机一点时间准备数据
  if (sWire->requestFrom((uint8_t)MAG_ADDR, (uint8_t)n, (bool)true) != n) {
    i2cErr(6);
    return false;
  }
  for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)sWire->read();
  return true;
}

static bool readReg(uint8_t reg, uint8_t& val) {
  return readRegs(reg, &val, 1);
}

/* ==================== 换算 ==================== */
/* 原始计数 -> µT（还没做修正） */
static inline float rawToUT(int16_t raw) {
  return (float)raw * 100.0f / LSB_PER_GAUSS[sGain & 0x07];   // 1 Gauss = 100 µT
}

/* 原始计数 -> 修正后的 µT */
static inline float fixX(int16_t raw) { return (rawToUT(raw) - sOffX) * sScX; }
static inline float fixY(int16_t raw) { return (rawToUT(raw) - sOffY) * sScY; }
static inline float fixZ(int16_t raw) { return (rawToUT(raw) - sOffZ) * sScZ; }

static inline float norm360(float a) {
  while (a < 0.0f)     a += 360.0f;
  while (a >= 360.0f)  a -= 360.0f;
  return a;
}

static inline float norm180(float a) {
  a = norm360(a);
  return (a > 180.0f) ? (a - 360.0f) : a;
}

/* 由 X/Y 分量算航向角：0=北，顺时针 0~360 */
static float headingFrom(float x, float y) {
  return norm360(atan2f(y, x) * 180.0f / PI_F + sDecl);
}

/* ==================== 配置 ==================== */
static void loadConfigFromFlash() {
  sPrefs.begin("hangxiang", false);      // 命名空间一次开好，后面一直用
  sAvg  = (int8_t)sPrefs.getInt("avg",  MAG_AVG);
  sRate = (int8_t)sPrefs.getInt("rate", MAG_RATE);
  sGain = (int8_t)sPrefs.getInt("gain", MAG_GAIN);
  sMode = (int8_t)sPrefs.getInt("mode", MAG_MODE);
  sDecl = sPrefs.getFloat("decl", MAG_DECL_DEG);
  if (sPrefs.isKey("ox")) {
    sOffX = sPrefs.getFloat("ox", 0);   sOffY = sPrefs.getFloat("oy", 0);
    sOffZ = sPrefs.getFloat("oz", 0);
    sScX  = sPrefs.getFloat("sx", 1);   sScY  = sPrefs.getFloat("sy", 1);
    sScZ  = sPrefs.getFloat("sz", 1);
    sCalFromFlash = true;
  }
  if (sPrefs.isKey("zero")) {
    sZeroMagDeg = sPrefs.getFloat("zero", 0);
    sZeroSet    = true;
  }
}

static void applyChipConfig() {
  sAvg  = (int8_t)constrain((int)sAvg, 0, 3);
  sRate = (int8_t)constrain((int)sRate, 0, 7);
  sGain = (int8_t)constrain((int)sGain, 0, 7);
  sMode = (int8_t)constrain((int)sMode, 0, 2);

  /* CRA: MA1 MA0 | DO2 DO1 DO0 | MS1 MS0 */
  writeReg(REG_CRA, (uint8_t)(((sAvg & 0x03) << 5) | ((sRate & 0x07) << 2)));
  /* CRB: GN2 GN1 GN0 */
  writeReg(REG_CRB, (uint8_t)((sGain & 0x07) << 5));
  /* MR: 连续=0x00 单次=0x01 空闲=0x02 */
  writeReg(REG_MR, (uint8_t)(sMode & 0x03));

  /* 读节奏：输出速率 → 间隔，再按平均次数放大。
   * 平均次数是"芯片内部攒够 N 次采样才出一个结果"，攒的过程中被读会
   * 读不到（表现为偶发 NACK、errors 一直涨、数值卡住不动），所以间隔
   * 要跟着平均次数一起放大，别读得比出数据还快。 */
  sReadInterval = RATE_MS[sRate];
  if (sReadInterval < MAG_READ_MS) sReadInterval = MAG_READ_MS;
  if (sAvg > 0) sReadInterval *= (1 << sAvg);
}

/* ==================== 认芯片 ==================== */
static bool probeChip() {
  uint8_t id[3];
  if (!readRegs(REG_IDA, id, 3)) {
    /* 偶尔读失败是正常的（尤其 400kHz + 杜邦线），攒够 3 次才认掉线，
     * 免得一帧失败就把"在线"翻成"离线"，让 selftest 误报没接线 */
    if (++sProbeFails >= 3) {
      sPresent = false;
      sIdOk    = false;
    }
    return false;
  }
  sProbeFails = 0;
  sIdA = id[0]; sIdB = id[1]; sIdC = id[2];
  sPresent = true;
  sIdOk    = (sIdA == 0x48 && sIdB == 0x34 && sIdC == 0x33);   // 'H' '4' '3'
  return true;
}

/* ==================== 生命周期 ==================== */
void magBegin() {
#if I2C_BUS_INDEX == 1
  /* 磁力计单独一路 I2C（不占激光那条总线），引脚在 config.h 里配 */
  sWire    = &Wire1;
  sBusName = "Wire1";
#endif
  /* 上电先量一下两条线的空闲电平：模块自带上拉的话两线都应是高。
   * 全是低/乱跳说明板上没有上拉或线没接好，这正是读数据不稳的头号原因。 */
  pinMode(I2C_SDA, INPUT);
  pinMode(I2C_SCL, INPUT);
  delay(2);
  sPullSda = digitalRead(I2C_SDA) ? 1 : 0;
  sPullScl = digitalRead(I2C_SCL) ? 1 : 0;

  /* 上拉体检：量两条线的上升时间，再据此定 I2C 初速。
   * 上拉偏弱时不用你手动改 config.h，代码自己就会降速跑。 */
  sRiseSdaUs = measureRiseUs(I2C_SDA);
  sRiseSclUs = measureRiseUs(I2C_SCL);
  {
    uint32_t worst = (sRiseSdaUs > sRiseSclUs) ? sRiseSdaUs : sRiseSclUs;
    sSpeedStep = (int8_t)stepForRise(worst);
    sBusHz     = speedFor(sSpeedStep);
  }

  /* 模块自带上拉是最理想的；万一碰上没焊上拉的批次，这里并联一点内部上拉救急
   * （ESP32 内部上拉约 45k，偏弱，只能兜底，真正治本还是外接 4.7k 到 3V3） */
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  sWire->begin(I2C_SDA, I2C_SCL, I2C_HZ);
  sWire->setTimeOut(100);              // 上拉弱时边沿慢，超时放宽到 100ms 免得误判
  if (sBusHz != I2C_HZ) {
    sWire->setClock(sBusHz);
    Serial.printf("[I2C] 上拉偏弱（SDA 上升 %luµs / SCL %luµs），时钟自动降到 %lu Hz\n",
                  (unsigned long)sRiseSdaUs, (unsigned long)sRiseSclUs,
                  (unsigned long)sBusHz);
  }
  loadConfigFromFlash();
  probeChip();
  if (sPresent) applyChipConfig();
  sLastReadMs  = millis();
  sLastProbeMs = millis();
  sLastGoodMs  = millis();
}

/* 读一帧原始数据：注意顺序是 X、Z、Y */
static bool readRawFrame(int16_t& x, int16_t& y, int16_t& z) {
  uint8_t d[6];
  if (!readRegs(REG_DATA, d, 6)) return false;
  x = (int16_t)((d[0] << 8) | d[1]);     // 0x03/0x04
  z = (int16_t)((d[2] << 8) | d[3]);     // 0x05/0x06  <- 这里是 Z
  y = (int16_t)((d[4] << 8) | d[5]);     // 0x07/0x08  <- 这里是 Y
  return true;
}

/* 单次测量：触发一次 → 轮询状态寄存器的 RDY 位 → 读数据。
 * 这是 HMC5883L/HMC5983 数据手册推荐的用法，不用猜芯片什么时候在忙。 */
static bool readSampleSingle(int16_t& x, int16_t& y, int16_t& z) {
  if (!writeReg(REG_MR, 0x01)) return false;        // MD=01：测一次就回空闲
  delay(3);                                         // 让它先测一会儿再开始问

  unsigned long t0 = millis();
  uint8_t sr = 0;
  bool ready = false;
  while (millis() - t0 < 30UL) {                    // 最多等 30ms
    if (!readReg(REG_SR, sr)) return false;
    if (sr & 0x01) { ready = true; break; }         // bit0 = RDY
    delay(1);
  }
  if (!ready) return false;
  return readRawFrame(x, y, z);
}

/* 当前配置下取一帧：连续模式直接读，单次模式先触发再等就绪 */
static bool readSample(int16_t& x, int16_t& y, int16_t& z) {
  if (sMode == 1) return readSampleSingle(x, y, z);
  return readRawFrame(x, y, z);
}

void magUpdate() {
  unsigned long now = millis();

  /* 诊断用：loop 每轮都会调到这里，间隔突然变大说明别的环节在阻塞 */
  if (sPrevUpdateMs) {
    uint32_t gap = (uint32_t)(now - sPrevUpdateMs);
    if (gap > sGapMaxMs) sGapMaxMs = gap;
  }
  sPrevUpdateMs = now;

  /* 掉线了别死等：每 2 秒重认一次，插回去能自己恢复 */
  if (!sPresent) {
    if (now - sLastProbeMs >= 2000UL) {
      sLastProbeMs = now;
      probeChip();
      if (sPresent) {
        applyChipConfig();
        sLastReadMs = now;
      }
    }
    return;
  }

  if (now - sLastReadMs < sReadInterval) return;
  sLastReadMs = now;

  int16_t rx, ry, rz;
  unsigned long t0 = micros();
  bool got = readSample(rx, ry, rz);
  if (!got) got = readSample(rx, ry, rz);        // 失败先重试一次
  uint32_t dt = (uint32_t)((micros() - t0) / 1000UL);
  if (dt > sReadMaxMs) sReadMaxMs = dt;

  /* 每 20 次读统计一次失败率：超过一半说明当前时钟对这条总线还是太快，
   * 自动再降一档。真解决还是要外接 4.7k 上拉，这里只是让它先能用。 */
  sWinTotal++;
  if (!got) sWinFail++;
  if (sWinTotal >= 20) {
    if ((uint16_t)sWinFail * 2 > sWinTotal && sSpeedStep < 3) {
      sSpeedStep++;
      sBusHz = speedFor(sSpeedStep);
      sWire->setClock(sBusHz);
      sSpeedMoves++;
      Serial.printf("[I2C] 最近 20 次失败 %u 次 → 时钟自动降到 %lu Hz（上拉偏弱，建议外接 4.7k）\n",
                    (unsigned)sWinFail, (unsigned long)sBusHz);
    }
    sWinTotal = 0;
    sWinFail  = 0;
  }

  if (!got) {
    sErrors++;
    if (++sConsecFails >= 20) {         // 连着 20 次都不通才复位总线，别把
      delay(1);                         // 偶发毛刺放大成一次总线重建
      sConsecFails = 0;
      i2cRecover();
    }
    if (sErrors % 10 == 0) probeChip();   // 攒几次失败再查一下还在不在
    return;
  }
  sConsecFails = 0;
  sLastGoodMs  = now;
  sRawX = rx; sRawY = ry; sRawZ = rz;
  sSamples++;

  /* 标定期间先记未修正的极值，标定值不能参与，否则会自己追自己 */
  if (sCalRunning) {
    float ux = rawToUT(rx), uy = rawToUT(ry), uz = rawToUT(rz);
    if (!sCalHasSample) {
      sCalMinX = sCalMaxX = ux;
      sCalMinY = sCalMaxY = uy;
      sCalMinZ = sCalMaxZ = uz;
      sCalHasSample = true;
    } else {
      if (ux < sCalMinX) sCalMinX = ux;
      if (ux > sCalMaxX) sCalMaxX = ux;
      if (uy < sCalMinY) sCalMinY = uy;
      if (uy > sCalMaxY) sCalMaxY = uy;
      if (uz < sCalMinZ) sCalMinZ = uz;
      if (uz > sCalMaxZ) sCalMaxZ = uz;
    }
    if (now >= sCalEndMs) magCalStop(sCalMessage);
  }

  float x = fixX(rx), y = fixY(ry), z = fixZ(rz);

  /* 对分量做指数平滑再算角度：直接平滑角度会在 0°/360° 接缝处来回跳 */
  if (!sSmoothReady) {
    sXs = x; sYs = y; sZs = z;
    sSmoothReady = true;
  } else {
    sXs += MAG_EMA_ALPHA * (x - sXs);
    sYs += MAG_EMA_ALPHA * (y - sYs);
    sZs += MAG_EMA_ALPHA * (z - sZs);
  }
  sX = sXs; sY = sYs; sZ = sZs;
  sHeadingRaw = headingFrom(x, y);
  sHeading    = headingFrom(sXs, sYs);
}

/* ==================== 读数 ==================== */
bool     magPresent()      { return sPresent; }
bool     magIdOk()         { return sIdOk; }
float    magX()            { return sX; }
float    magY()            { return sY; }
float    magZ()            { return sZ; }
float    magHeadingDeg()   { return sHeading; }
float    magHeadingRawDeg(){ return sHeadingRaw; }

bool  magZeroSet()         { return sZeroSet; }
float magZeroDeg()         { return sZeroSet ? norm360(sZeroMagDeg + sDecl) : 0.0f; }

float magHeadingRel() {
  if (!sZeroSet) return sHeading;          // 没设出发点就是绝对航向
  return norm360(sHeading - sDecl - sZeroMagDeg);
}

float magDeviation() {
  if (!sZeroSet) return 0.0f;
  return norm180(sHeading - sDecl - sZeroMagDeg);
}

bool magZeroHere(String& msg) {
  if (sSamples == 0) {          // 只要读到过数据就能定出发点，不看此刻的在线标志
    msg = "[ZERO] 还没读到有效数据，等一帧再来";
    return false;
  }
  sZeroMagDeg = norm360(sHeading - sDecl);
  sZeroSet    = true;
  sPrefs.putFloat("zero", sZeroMagDeg);
  char b[320];
  snprintf(b, sizeof(b),
           "[ZERO] 已把当前朝向设为出发点：相对航向 0.0°（绝对 %.1f°）。\n"
           "[ZERO] 之后 data、网页、/api/mag 的 heading_rel_deg 都相对这个方向，\n"
           "[ZERO] dev_deg 是偏差：正数偏右（顺时针），负数偏左。断电不丢。",
           magZeroDeg());
  msg = String(b);
  return true;
}

bool magZeroClear(String& msg) {
  if (!sZeroSet) {
    msg = "[ZERO] 本来就没设过出发点";
    return false;
  }
  sZeroSet = false;
  sZeroMagDeg = 0.0f;
  sPrefs.remove("zero");
  msg = "[ZERO] 已清除出发点，航向恢复成相对磁北的绝对角度";
  return true;
}
int      magRawX()         { return sRawX; }
int      magRawY()         { return sRawY; }
int      magRawZ()         { return sRawZ; }
uint32_t magSamples()      { return sSamples; }
uint32_t magErrors()       { return sErrors; }

uint32_t magStaleMs() {
  if (sSamples == 0) return millis();       // 一帧都没读到，按上电时长算
  return millis() - sLastGoodMs;
}

String magDiagText() {
  char b[260];
  snprintf(b, sizeof(b),
           "[I2C] 失败 %lu 次 → 地址无应答 %u | 数据无应答 %u | 总线超时 %u | "
           "读不到字节 %u | 其它 %u；总线复位 %lu 次；最后错误码 %u",
           (unsigned long)sErrors,
           sErrCode[2], sErrCode[3], sErrCode[5], sErrCode[6],
           (unsigned)(sErrCode[1] + sErrCode[4]),
           (unsigned long)sRecovers, (unsigned)sLastErrCode);
  return String(b);
}

String magPullupText() {
  char b[220];
  snprintf(b, sizeof(b),
           "[I2C] 上电时空闲电平 SDA=%s SCL=%s（都应为高。有低的就是没上拉/"
           "线没接好，建议模块 3V3 到 SDA/SCL 各加一只 4.7k 上拉）",
           sPullSda == 2 ? "?" : (sPullSda ? "高" : "低！"),
           sPullScl == 2 ? "?" : (sPullScl ? "高" : "低！"));
  return String(b);
}

uint32_t magBusHz() { return sBusHz; }
const char* magBusName() { return sBusName; }

String magRiseText() {
  char b[240];
  const char* judge = (sSpeedStep == 0) ? "上拉够硬，可以全速跑"
                    : (sSpeedStep == 1) ? "上拉偏弱，已降半速"
                    : (sSpeedStep == 2) ? "上拉很弱，已降到 1/5 速"
                                        : "上拉极弱，已降到 1/10 速";
  snprintf(b, sizeof(b),
           "[I2C] 上拉体检：SDA 上升 %lu µs，SCL 上升 %lu µs（规范要求 100kHz<1µs、"
           "400kHz<0.3µs）→ %s，当前 %lu Hz，自动降速 %lu 次",
           (unsigned long)sRiseSdaUs, (unsigned long)sRiseSclUs,
           judge, (unsigned long)sBusHz, (unsigned long)sSpeedMoves);
  return String(b);
}

String magBusLevelText() {
  /* Wire 用开漏方式驱动这两条线，GPIO 的输入通路一直是通的，
   * 所以直接 digitalRead 就能看到总线当前被谁拉着。 */
  int sda = digitalRead(I2C_SDA);
  int scl = digitalRead(I2C_SCL);
  char b[240];
  snprintf(b, sizeof(b),
           "[I2C] 此刻总线电平 SDA=%s SCL=%s（两个都该是高。有低的就是总线被"
           "拉死，读数会一直卡着直到放开）",
           sda ? "高" : "低！", scl ? "高" : "低！");
  return String(b);
}

String magTimingText() {
  char b[200];
  snprintf(b, sizeof(b),
           "[TIME] 读一帧最慢 %lu ms（正常应 <5ms），magUpdate 间隔最大 %lu ms"
           "（正常 <10ms，读周期 %lu ms）",
           (unsigned long)sReadMaxMs, (unsigned long)sGapMaxMs,
           (unsigned long)sReadInterval);
  return String(b);
}
int      magGain()         { return sGain; }
int      magRate()         { return sRate; }
int      magAverage()      { return 1 << sAvg; }
float    magDeclination()  { return sDecl; }

String magModeText() {
  return (sMode == 0) ? String("连续") : (sMode == 1 ? String("单次") : String("空闲"));
}

float magFieldUT() {
  return sqrtf(sX * sX + sY * sY + sZ * sZ);
}

String magChipIdText() {
  char b[64];
  if (!sPresent) return String("无应答");
  snprintf(b, sizeof(b), "%c%c%c (0x%02X 0x%02X 0x%02X)",
           (char)sIdA, (char)sIdB, (char)sIdC, sIdA, sIdB, sIdC);
  return String(b);
}

float magTemperatureC(bool& ok) {
  ok = false;
  if (!sPresent || !MAG_TEMP_ENABLE) return 0.0f;
  if (!writeReg(REG_TEMP, 0x80)) return 0.0f;      // 置 TEMP_EN，启动一次温度测量
  delay(10);
  uint8_t b[2];
  if (!readRegs(REG_TEMP, b, 2)) return 0.0f;
  int16_t raw = (int16_t)(((uint16_t)b[0] << 8) | (uint16_t)(b[1] & 0xC0));
  raw = (int16_t)(raw >> 6);
  ok = true;
  return raw / 8.0f + MAG_TEMP_OFFSET_C;
}

/* ==================== 标定 ==================== */
bool magCalAutoStart(uint32_t seconds, String& msg) {
  if (!sPresent) {
    msg = "[CAL] 磁力计没应答，先解决接线再标定（scan 命令可以扫总线）";
    return false;
  }
  if (seconds == 0) seconds = MAG_CAL_DEFAULT_SEC;
  sCalRunning   = true;
  sCalFinished  = false;
  sCalHasSample = false;
  sCalEndMs     = millis() + seconds * 1000UL;
  msg  = "[CAL] 开始标定，持续 " + String(seconds) + " 秒。\n";
  msg += "[CAL] 请把模块拿在手里，离开铁桌/电机/磁铁，\"8\" 字形慢慢转，\n";
  msg += "[CAL] 三个方向都要转到（平放转一圈、竖起来再转一圈）。";
  return true;
}

static bool finishCalibration(String& msg) {
  if (!sCalHasSample) {
    msg = "[CAL] 一帧数据都没采到，标定作废";
    return false;
  }
  float rx = sCalMaxX - sCalMinX;
  float ry = sCalMaxY - sCalMinY;
  float rz = sCalMaxZ - sCalMinZ;

  if (rx < MAG_CAL_MIN_RANGE_UT || ry < MAG_CAL_MIN_RANGE_UT ||
      rz < MAG_CAL_MIN_RANGE_UT) {
    char b[260];
    snprintf(b, sizeof(b),
             "[CAL] 标定失败：三个轴的变化量只有 X %.1f / Y %.1f / Z %.1f µT，"
             "至少都要 %.1f µT。\n[CAL] 多半是没转够或转得太快，"
             "再 cal auto 一次，慢一点、每个方向都转到。",
             rx, ry, rz, (double)MAG_CAL_MIN_RANGE_UT);
    msg = String(b);
    sCalRunning = false;
    return false;
  }

  sOffX = (sCalMaxX + sCalMinX) / 2.0f;
  sOffY = (sCalMaxY + sCalMinY) / 2.0f;
  sOffZ = (sCalMaxZ + sCalMinZ) / 2.0f;
  float avgR = (rx + ry + rz) / 3.0f;
  sScX = avgR / rx;
  sScY = avgR / ry;
  sScZ = avgR / rz;

  sCalRunning   = false;
  sCalFinished  = true;
  sCalFromFlash = true;
  sCalSavedAtMs = millis();

  float ox = sOffX, oy = sOffY, oz = sOffZ;
  sPrefs.putFloat("ox", ox); sPrefs.putFloat("oy", oy); sPrefs.putFloat("oz", oz);
  sPrefs.putFloat("sx", sScX); sPrefs.putFloat("sy", sScY); sPrefs.putFloat("sz", sScZ);
  bool zeroCleared = zeroInvalidate();

  char b[320];
  snprintf(b, sizeof(b),
           "[CAL] 标定完成并存好了（断电不丢）。\n"
           "[CAL] 硬磁偏移 X %.1f  Y %.1f  Z %.1f µT\n"
           "[CAL] 软磁比例 X %.3f  Y %.3f  Z %.3f\n"
           "[CAL] 轴变化量 X %.1f  Y %.1f  Z %.1f µT",
           sOffX, sOffY, sOffZ, sScX, sScY, sScZ, rx, ry, rz);
  msg = String(b);
  if (zeroCleared)
    msg += "\n[CAL] 标定改变了磁北系，原来的出发点已清除，请重新敲 zero 定一次";
  return true;
}

bool magCalStop(String& msg) {
  if (!sCalRunning) {
    msg = "[CAL] 当前没有在标定";
    return false;
  }
  return finishCalibration(msg);
}

bool magCalCancel(String& msg) {
  if (!sCalRunning) {
    msg = "[CAL] 当前没有在标定";
    return false;
  }
  sCalRunning = false;
  msg = "[CAL] 已放弃本次标定，原来的标定值保持不变";
  return true;
}

bool magCalSetOffsets(float ox, float oy, float oz, String& msg) {
  sOffX = ox; sOffY = oy; sOffZ = oz;
  sCalFromFlash = true;
  sPrefs.putFloat("ox", ox); sPrefs.putFloat("oy", oy); sPrefs.putFloat("oz", oz);
  bool zeroCleared = zeroInvalidate();
  char b[120];
  snprintf(b, sizeof(b), "[CAL] 硬磁偏移已设为 X %.1f  Y %.1f  Z %.1f µT（已存 flash）",
           ox, oy, oz);
  msg = String(b);
  if (zeroCleared) msg += "\n[CAL] 出发点已一并清除（磁北系变了），请重新敲 zero";
  return true;
}

bool magCalSetScale(float sx, float sy, float sz, String& msg) {
  if (sx <= 0 || sy <= 0 || sz <= 0) {
    msg = "[CAL] 比例必须是正数";
    return false;
  }
  sScX = sx; sScY = sy; sScZ = sz;
  sPrefs.putFloat("sx", sx); sPrefs.putFloat("sy", sy); sPrefs.putFloat("sz", sz);
  bool zeroCleared = zeroInvalidate();
  char b[120];
  snprintf(b, sizeof(b), "[CAL] 软磁比例已设为 X %.3f  Y %.3f  Z %.3f（已存 flash）", sx, sy, sz);
  msg = String(b);
  if (zeroCleared) msg += "\n[CAL] 出发点已一并清除（磁北系变了），请重新敲 zero";
  return true;
}

bool magCalReset(String& msg) {
  sOffX = sOffY = sOffZ = 0.0f;
  sScX  = sScY  = sScZ  = 1.0f;
  sCalFromFlash = false;
  sPrefs.remove("ox"); sPrefs.remove("oy"); sPrefs.remove("oz");
  sPrefs.remove("sx"); sPrefs.remove("sy"); sPrefs.remove("sz");
  msg = "[CAL] 已清除标定值，回到工厂零偏。指北会偏，重新 cal auto 即可";
  if (zeroInvalidate())
    msg += "\n[CAL] 出发点已一并清除（磁北系变了），请重新敲 zero";
  return true;
}

bool   magCalibrating()      { return sCalRunning; }
bool   magCalibrated()       { return sCalFromFlash; }
bool   magCalJustFinished()  { bool v = sCalFinished; sCalFinished = false; return v; }
String magCalLastMessage()   { return sCalMessage; }

uint32_t magCalRemainingSec() {
  if (!sCalRunning) return 0;
  unsigned long now = millis();
  if (now >= sCalEndMs) return 0;
  return (uint32_t)((sCalEndMs - now + 999UL) / 1000UL);
}

/* ==================== 参数设置 ==================== */
bool magSetDeclination(float deg, String& msg) {
  if (deg < -180.0f || deg > 180.0f) {
    msg = "[CFG] 磁偏角要在 -180 ~ 180 度之间";
    return false;
  }
  sDecl = deg;
  sPrefs.putFloat("decl", deg);
  char b[140];
  snprintf(b, sizeof(b),
           "[CFG] 磁偏角已设为 %.2f°（已存 flash）。航向角现在按真北输出", deg);
  msg = String(b);
  return true;
}

bool magSetGain(int g, String& msg) {
  if (g < 0 || g > 7) { msg = "[CFG] 量程档位 0~7，例：gain 1"; return false; }
  sGain = (int8_t)g;
  sPrefs.putInt("gain", g);
  applyChipConfig();
  char b[140];
  snprintf(b, sizeof(b), "[CFG] 量程设为 %s（灵敏度 %.0f LSB/Gauss，已存 flash）",
           GAIN_NAME[g], (double)LSB_PER_GAUSS[g]);
  msg = String(b);
  return true;
}

bool magSetRate(int r, String& msg) {
  if (r < 0 || r > 7) { msg = "[CFG] 速率档位 0~7，例：rate 6"; return false; }
  sRate = (int8_t)r;
  sPrefs.putInt("rate", r);
  applyChipConfig();
  char b[140];
  snprintf(b, sizeof(b), "[CFG] 输出速率设为 %s（每 %lu ms 读一次）",
           RATE_NAME[r], (unsigned long)sReadInterval);
  msg = String(b);
  return true;
}

bool magSetAverage(int a, String& msg) {
  if (a < 0 || a > 3) { msg = "[CFG] 平均次数档位 0~3（1/2/4/8 次），例：avg 3"; return false; }
  sAvg = (int8_t)a;
  sPrefs.putInt("avg", a);
  applyChipConfig();
  char b[120];
  snprintf(b, sizeof(b), "[CFG] 采样平均设为 %d 次（已存 flash）", 1 << a);
  msg = String(b);
  return true;
}

bool magSaveConfig(String& msg) {
  sPrefs.putInt("avg", sAvg);
  sPrefs.putInt("rate", sRate);
  sPrefs.putInt("gain", sGain);
  sPrefs.putInt("mode", sMode);
  sPrefs.putFloat("decl", sDecl);
  msg = "[CFG] 当前配置和标定值都已写入 flash";
  return true;
}

/* ==================== 自检 / 扫描 ==================== */
bool magSelfTest(String& msg) {
  probeChip();                  // 先把状态刷新一下，别拿几秒前的标志下结论
  String out;
  out += "[TEST] 识别寄存器：";
  out += magChipIdText();
  out += sIdOk ? "  → 是 HMC5983/HMC5883L 系列，认到了\n"
               : "  → 不是 \"H43\"，可能是别的磁力计或接线接触不良\n";

  if (!sPresent) {
    out += "[TEST] I2C 上没有应答（连认 3 次都没回）。按顺序查：\n"
           "[TEST]   1) VCC/GND 是否接好，模块有没有电\n"
           "[TEST]   2) SDA->GPIO21、SCL->GPIO22 有没有接反\n"
           "[TEST]   3) 用 scan 命令扫一遍总线，看 0x1E 在不在\n"
           "[TEST]   4) 400kHz 配长杜邦线容易失败，config.h 里 I2C_HZ 改成 100000";
    msg = out;
    return false;
  }

  int n = 0;
  float sum = 0;
  for (int i = 0; i < 8; i++) {
    magUpdate();
    delay(30);
    if (magSamples() == 0) continue;
    sum += magFieldUT();
    n++;
  }
  float avg = (n > 0) ? sum / n : 0.0f;
  char b[200];
  snprintf(b, sizeof(b),
           "[TEST] 连读 8 次，平均磁场强度 %.1f µT（地面一般 25~65 µT）\n", avg);
  out += b;

  if (n == 0) {
    out += "[TEST] 8 次一次都没读到：总线不稳，把 config.h 的 I2C_HZ 降到 100000，"
           "并检查杜邦线有没有插紧、线尽量短\n";
  }
  if (abs(sRawX) >= 2040 || abs(sRawY) >= 2040 || abs(sRawZ) >= 2040)
    out += "[TEST] 有轴读数顶到 ±2048：量程太小或旁边有磁铁，用 gain 4 换个大量程再试\n";
  if (avg < 10.0f)
    out += "[TEST] 读数太小：先 cal reset 去掉旧标定，再放到空地上重测\n";
  if (!magCalibrated())
    out += "[TEST] 还没标定。航向能用但会有几十度偏差，建议 cal auto 15 转一圈\n";
  else
    out += "[TEST] 已有标定值，指北应当基本正确（磁偏角记得设）\n";
  msg = out;
  return true;
}

String magScanI2C(String& msg) {
  String found;
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    sWire->beginTransmission(a);
    if (sWire->endTransmission() == 0) {
      char b[16];
      snprintf(b, sizeof(b), "0x%02X ", a);
      found += b;
      n++;
    }
  }
  String out;
  if (n == 0) {
    char hdr[96];
    snprintf(hdr, sizeof(hdr), "[I2C] %s 上一个设备都没扫到（SDA=GPIO%d，SCL=GPIO%d）：",
             sBusName, I2C_SDA, I2C_SCL);
    out = String(hdr) + "接反、没共地、或模块没供电";
  } else {
    out = "[I2C] 扫到 " + String(n) + " 个地址：" + found;
    out += (found.indexOf("0x1E") >= 0) ? "\n[I2C] 0x1E = HMC5983 磁力计，接线没问题"
                                        : "\n[I2C] 没看到 0x1E，检查 SDA/SCL 有没有接反";
    if (found.indexOf("0x29") >= 0)
      out += "（0x29 是 VL53L1X 激光测距，挂在同一条总线上的另一个器件，正常）";
  }
  msg = out;
  return found;
}

/* ==================== JSON ==================== */
String magJson() {
  char b[640];
  bool tempOk = false;
  float temp = magTemperatureC(tempOk);

  char tempPart[48];
  if (tempOk) snprintf(tempPart, sizeof(tempPart), "%.1f", temp);
  else        snprintf(tempPart, sizeof(tempPart), "null");

  snprintf(b, sizeof(b),
           "{\"ok\":%s,\"present\":%s,\"id_ok\":%s,\"chip\":\"%c%c%c\","
           "\"heading_deg\":%.1f,\"heading_raw_deg\":%.1f,\"decl_deg\":%.2f,"
           "\"heading_rel_deg\":%.1f,\"dev_deg\":%.1f,\"zero_set\":%s,\"zero_deg\":%.1f,"
           "\"x_ut\":%.2f,\"y_ut\":%.2f,\"z_ut\":%.2f,\"field_ut\":%.2f,"
           "\"raw\":[%d,%d,%d],\"temp_c\":%s,"
           "\"gain\":%d,\"rate\":%d,\"avg\":%d,\"calibrated\":%s,"
           "\"calibrating\":%s,\"cal_remain_s\":%lu,\"samples\":%lu,\"errors\":%lu,"
           "\"stale_ms\":%lu,"
           "\"uptime_s\":%lu}",
           (sPresent && sSamples > 0) ? "true" : "false",
           sPresent ? "true" : "false",
           sIdOk ? "true" : "false",
           (char)(sIdA ? sIdA : '?'), (char)(sIdB ? sIdB : '?'), (char)(sIdC ? sIdC : '?'),
           sHeading, sHeadingRaw, sDecl,
           magHeadingRel(), magDeviation(), sZeroSet ? "true" : "false", magZeroDeg(),
           sX, sY, sZ, magFieldUT(),
           sRawX, sRawY, sRawZ, tempPart,
           (int)sGain, (int)sRate, 1 << sAvg, sCalFromFlash ? "true" : "false",
           sCalRunning ? "true" : "false",
           (unsigned long)magCalRemainingSec(),
           (unsigned long)sSamples, (unsigned long)sErrors,
           (unsigned long)magStaleMs(),
           millis() / 1000UL);
  return String(b);
}
