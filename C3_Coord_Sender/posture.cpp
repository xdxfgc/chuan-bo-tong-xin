/* =====================================================================
   posture.cpp   姿态判定实现（MPU6050）
   ---------------------------------------------------------------------
   这个版本重点是**采集数据**，判定逻辑留了口子但默认不参与上报
   （POSTURE_JUDGE = 0），因为阈值必须用实测数据标定，不能拍脑袋。

   标定做法：把串口数据分别记四组各一二十秒——
     站着不动 / 走一走 / 往身上泼水 / 装袋放水盆里漂着
   四组摆一起看，阈值该定在哪就一目了然了。
   ===================================================================== */

#include "posture.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

static Adafruit_MPU6050 mpu;
static bool          s_ready     = false;
static unsigned long s_lastReadMs = 0;
static unsigned long s_lastPrintMs = 0;

/* ---------------- 当前值 ---------------- */
static float s_ax = 0.0f, s_ay = 0.0f, s_az = 0.0f;   // 加速度（g）
static float s_gx = 0.0f, s_gy = 0.0f, s_gz = 0.0f;   // 角速度（度每秒）
static float s_accG  = 1.0f;                          // 合加速度（g）
static float s_pitch = 0.0f, s_roll = 0.0f;           // 姿态角（度）
static float s_temp  = 0.0f;

/* ---------------- 上电基准姿态 ----------------
   救生衣的佩戴方向每个人都不一样，所以不能比绝对角度，
   只能比"相对刚上电时转过了多少度"。                              */
static float s_basePitch = 0.0f, s_baseRoll = 0.0f;
static bool  s_baseSet   = false;

/* ---------------- 运动强度 ----------------
   记半秒窗口内合加速度的峰峰值。走路时起伏大，漂在水面上就很小。   */
#define MOTION_WIN 25                       // 25 × 20ms = 500ms
static float s_gWin[MOTION_WIN];
static int   s_gIdx = 0;
static int   s_gCnt = 0;

/* ---------------- 入水冲击波形 ----------------
   真落水：先失重（<0.6g），紧接着冲击（>1.8g）。
   被浪打到身上几乎不会出现这个组合。                              */
static bool          s_freefall    = false;
static unsigned long s_freefallMs  = 0;
static bool          s_impactSeen  = false;
static unsigned long s_lastImpactMs = 0;

/* ---------------- 摇摆频率（只用于打印） ----------------
   每 100ms 记一个横滚角，凑成 8 秒窗口，数它过均值的次数。
   走路是 1~2Hz 的步态节奏，波浪只有 0.1~0.5Hz，频率上分得开。       */
#define SWAY_SAMPLE_MS 100
#define SWAY_BUF_N     (POSTURE_SWAY_WIN_MS / SWAY_SAMPLE_MS)
static float         s_swayBuf[SWAY_BUF_N];
static int           s_swayIdx    = 0;
static int           s_swayCnt    = 0;
static unsigned long s_lastSwayMs = 0;

/* ---------------- 演示用的强制开关 ---------------- */
static bool s_simForced = false;

/* =====================================================================
   初始化
   ===================================================================== */

void postureBegin() {
  s_ready = false;

  Wire.begin(POSTURE_SDA_PIN, POSTURE_SCL_PIN);
  Wire.setClock(400000);

  if (!mpu.begin(POSTURE_ADDR, &Wire)) {
    DBG.println("MPU6050 初始化失败！按顺序查这四处：");
    DBG.println("  1) 3V3 有没有电（只能接 3.3V，接 5V 会烧）");
    DBG.println("  2) GND 有没有和 C3 共地");
    DBG.printf ("  3) SDA 是不是接在 GPIO%d、SCL 是不是接在 GPIO%d（别接反）\n",
                POSTURE_SDA_PIN, POSTURE_SCL_PIN);
    DBG.println("  4) A0 有没有接 GND（接 GND 才是 0x68；悬空是 0x69，就要改 POSTURE_ADDR）");
    return;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  s_ready = true;
  DBG.printf("MPU6050 已就绪：地址 0x%02X  SDA=GPIO%d  SCL=GPIO%d  %luHz\n",
             POSTURE_ADDR, POSTURE_SDA_PIN, POSTURE_SCL_PIN,
             (unsigned long)(1000UL / POSTURE_READ_MS));
  DBG.println("          正在记录上电基准姿态，保持静止 1 秒…");

  /* 先连续读几次取平均当基准，避免第一个样本正好在抖动上 */
  delay(200);
  float sumP = 0.0f, sumR = 0.0f;
  const int N = 20;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    float ax = a.acceleration.x / 9.80665f;
    float ay = a.acceleration.y / 9.80665f;
    float az = a.acceleration.z / 9.80665f;
    sumR += atan2f(ay, az) * 180.0f / (float)M_PI;
    sumP += atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / (float)M_PI;
    delay(10);
  }
  s_basePitch = sumP / N;
  s_baseRoll  = sumR / N;
  s_baseSet   = true;

  DBG.printf("          基准姿态：俯仰 %.1f°  横滚 %.1f°\n", s_basePitch, s_baseRoll);
}

/* =====================================================================
   周期读取
   ===================================================================== */

void postureUpdate() {
  if (!s_ready) return;
  if (millis() - s_lastReadMs < POSTURE_READ_MS) return;
  unsigned long now = millis();
  s_lastReadMs = now;

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  s_ax = a.acceleration.x / 9.80665f;        // m/s² -> g
  s_ay = a.acceleration.y / 9.80665f;
  s_az = a.acceleration.z / 9.80665f;
  s_gx = g.gyro.x * 180.0f / (float)M_PI;    // rad/s -> 度每秒
  s_gy = g.gyro.y * 180.0f / (float)M_PI;
  s_gz = g.gyro.z * 180.0f / (float)M_PI;
  s_temp = t.temperature;

  /* 合加速度：静止时约 1.0。
     落水瞬间先掉到 0.6 以下（失重），再冲到 1.8 以上（入水冲击）。 */
  s_accG = sqrtf(s_ax * s_ax + s_ay * s_ay + s_az * s_az);

  /* 由重力分量算姿态角。MPU6050 没有磁力计，所以只有俯仰和横滚，
     偏航角不给（积分会漂）。 */
  s_roll  = atan2f(s_ay, s_az) * 180.0f / (float)M_PI;
  s_pitch = atan2f(-s_ax, sqrtf(s_ay * s_ay + s_az * s_az)) * 180.0f / (float)M_PI;

  /* 运动强度：半秒窗口的峰峰值 */
  s_gWin[s_gIdx] = s_accG;
  s_gIdx = (s_gIdx + 1) % MOTION_WIN;
  if (s_gCnt < MOTION_WIN) s_gCnt++;

  /* 入水冲击波形：失重 -> 冲击 */
  if (s_accG < POSTURE_FREEFALL_G) {
    if (!s_freefall) {
      s_freefall   = true;
      s_freefallMs = now;
    }
  } else if (s_freefall) {
    if (s_accG > POSTURE_IMPACT_G && (now - s_freefallMs) <= POSTURE_IMPACT_WIN_MS) {
      s_freefall     = false;
      s_impactSeen   = true;
      s_lastImpactMs = now;
      DBG.printf("[姿态] ★检测到入水冲击（合加速度 %.2f g）\n", s_accG);
    } else if ((now - s_freefallMs) > POSTURE_IMPACT_WIN_MS) {
      s_freefall = false;             // 失重之后太久没冲击，这次不算
    }
  }

  /* 摇摆频率的采样点（每 100ms 存一个横滚角） */
  if (now - s_lastSwayMs >= SWAY_SAMPLE_MS) {
    s_lastSwayMs = now;
    s_swayBuf[s_swayIdx] = s_roll;
    s_swayIdx = (s_swayIdx + 1) % SWAY_BUF_N;
    if (s_swayCnt < SWAY_BUF_N) s_swayCnt++;
  }
}

/* =====================================================================
   派生量
   ===================================================================== */

bool  postureReady()        { return s_ready; }
float postureAccelG()       { return s_accG; }
float posturePitchDeg()     { return s_pitch; }
float postureRollDeg()      { return s_roll; }
float postureTemperature()  { return s_temp; }
bool  postureImpactSeen()   { return s_impactSeen; }

/* 相对上电基准的倾角。俯仰和横滚合成一个角度，够用来判断"翻没翻"。 */
float postureTiltFromBase() {
  if (!s_baseSet) return 0.0f;
  float dp = s_pitch - s_basePitch;
  float dr = s_roll  - s_baseRoll;
  return sqrtf(dp * dp + dr * dr);
}

float postureMotionLevel() {
  if (s_gCnt < 2) return 0.0f;
  float mn = s_gWin[0], mx = s_gWin[0];
  for (int i = 1; i < s_gCnt; i++) {
    if (s_gWin[i] < mn) mn = s_gWin[i];
    if (s_gWin[i] > mx) mx = s_gWin[i];
  }
  return mx - mn;
}

/* ---------------- 三条子判据 ---------------- */

/* 判据①：入水冲击。
   光看"冲击"不行 —— 实测抖动能冲到 3.69g。必须要求**先失重、再冲击**，
   上浪拍打只有冲击、没有失重，这样就分开了。                        */
bool postureImpactRecent() {
  if (!s_impactSeen) return false;
  return (millis() - s_lastImpactMs) < POSTURE_IMPACT_KEEP_MS;
}

/* 判据②：姿态翻转。必须相对"上电基准"，不能用绝对角度 ——
   救生衣的佩戴方向每个人都不一样。                               */
bool postureFlipped() {
  return s_baseSet && (postureTiltFromBase() > POSTURE_TILT_MIN_DEG);
}

/* 判据③：平静。落水后人是漂着的，不会有持续的大幅运动。
   实测：抖动 1.3~4.2g；落水后应该小得多。                          */
bool postureCalm() {
  return postureMotionLevel() < POSTURE_CALM_MAX_G;
}

bool postureLooksLikeFloating() {
#if !POSTURE_JUDGE
  /* 姿态不参与判定（调传感器、或者想先把水感单独跑通时用）。
     这时三重确认实际上变成"水感 + 持续时间"两重。                */
  return true;
#else
  if (s_simForced) return true;
  if (!s_ready || !s_baseSet) return false;
  return postureImpactRecent() && postureFlipped() && postureCalm();
#endif
}

/* ---------------- 摇摆频率（打印用，不参与判定） ---------------- */

float postureSwayHz() {
  if (s_swayCnt < SWAY_BUF_N) return -1.0f;      // 8 秒窗口还没填满

  float sum = 0.0f;
  for (int i = 0; i < SWAY_BUF_N; i++) sum += s_swayBuf[i];
  float mean = sum / SWAY_BUF_N;

  /* 按时间顺序走（窗口满时最老的样本在 s_swayIdx） */
  int crossings = 0;
  bool wasPos = (s_swayBuf[s_swayIdx] >= mean);
  for (int k = 1; k < SWAY_BUF_N; k++) {
    int i = (s_swayIdx + k) % SWAY_BUF_N;
    bool isPos = (s_swayBuf[i] >= mean);
    if (isPos != wasPos) crossings++;
    wasPos = isPos;
  }

  float seconds = SWAY_BUF_N * SWAY_SAMPLE_MS / 1000.0f;
  return crossings / (2.0f * seconds);           // 一个完整周期过两次零
}

void postureSetSim(bool on) {
  s_simForced = on;
  if (on) {
    s_impactSeen   = true;
    s_lastImpactMs = millis();
  }
}

bool postureSimForced() { return s_simForced; }

/* =====================================================================
   数据行（标定阶段看这个）
   ===================================================================== */

void posturePrintReport() {
  if (!s_ready) return;
  if (millis() - s_lastPrintMs < POSTURE_PRINT_MS) return;
  s_lastPrintMs = millis();

  DBG.printf("[姿态] 合加速度 %.3f g  俯仰 %6.1f°  横滚 %6.1f°  "
             "相对基准 %5.1f°  运动 %.3f g  温度 %.1fC",
             s_accG, s_pitch, s_roll,
             postureTiltFromBase(), postureMotionLevel(), s_temp);

  float hz = postureSwayHz();
  if (hz < 0.0f) DBG.print("  摇摆 --");
  else           DBG.printf("  摇摆 %.2f Hz", hz);

  /* 三条判据的状态，标定和排查时一眼能看出卡在哪一条 */
  if (s_simForced)                DBG.print("  [模拟]");
  else if (postureLooksLikeFloating()) DBG.print("  [三条都满足]");
  else {
    DBG.print("  [");
    DBG.print(postureImpactRecent() ? "①有冲击 " : "①无冲击 ");
    DBG.print(postureFlipped()      ? "②已翻转 " : "②未翻转 ");
    DBG.print(postureCalm()         ? "③平静"   : "③在动");
    DBG.print("]");
  }
  DBG.println();
}
