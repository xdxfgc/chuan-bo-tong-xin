/* =====================================================================
   berth.cpp   靠泊辅助实现
   ---------------------------------------------------------------------
   处理链：原始距离 -> 3 点中值 -> 8 点平均 -> 滤波距离
                    滤波距离 -> 0.4 秒窗口最小二乘拟合 -> 接近速度
   滤波和拟合是必须的：激光噪声约 ±5mm，直接对相邻两帧求差，
   50ms 算出来的“速度”误差有 0.1 m/s，而提醒阈值才 0.15，噪声就能刷满告警。
   ===================================================================== */

#include "berth.h"
#include "tof.h"
#include "voice.h"

#if BERTH_ENABLE

#define MED_N 3          // 中值滤波点数
#define AVG_N 8          // 平均滤波点数
#define FIT_N 8          // 速度拟合窗口（8 × 50ms = 0.4 秒）
#define WIN_N 64         // 平稳/静默观察窗（64 × 50ms ≈ 3.2 秒）

static float    s_med[MED_N];
static int      s_medN = 0, s_medIdx = 0;

static float    s_avg[AVG_N];
static int      s_avgN = 0, s_avgIdx = 0;

static float    s_fitD[FIT_N];
static uint32_t s_fitT[FIT_N];
static int      s_fitN = 0, s_fitIdx = 0;

static float    s_win[WIN_N];
static int      s_winN = 0, s_winIdx = 0;

static float    s_dist      = -1.0f;   // 滤波后的距离
static float    s_lastDist  = -1.0f;   // 最近一次有效距离（失效率判断用）
static float    s_speed     =  0.0f;   // 接近速度（正=靠近）
static bool     s_valid     = false;

static bool     s_active    = false;   // 在靠泊监测中
static bool     s_docked    = false;   // 已靠妥
static uint8_t  s_alarm     = 0x00;
static uint8_t  s_candCode  = 0x00;    // 去抖：候选告警码
static uint8_t  s_candCnt   = 0;

static unsigned long s_lastSampleMs  = 0;
static unsigned long s_lastAnnounceMs = 0;
static unsigned long s_lastValidMs   = 0;
static unsigned long s_beyondMs      = 0;
static bool          s_beyondRun     = false;

static unsigned long s_enterMs       = 0;      // 进入监测的时刻
static float         s_maxDist       = 0.0f;   // 进入监测后见过的最大距离
static unsigned long s_undockMs      = 0;      // 开始远离的时刻

/* ---------------- 工具 ---------------- */

static float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// 观察窗内的极差（最大值 - 最小值），用来判断“稳没稳”
static float winRange() {
  // 观察窗没攒到一半时返回一个大值：表示"证据不足"，绝不能被当成"距离没变"
  if (s_winN < WIN_N / 2) return 999.0f;
  float mn = s_win[0], mx = s_win[0];
  for (int i = 0; i < s_winN; i++) {
    if (s_win[i] < mn) mn = s_win[i];
    if (s_win[i] > mx) mx = s_win[i];
  }
  return mx - mn;
}

// 用拟合窗里的 (时间, 距离) 做最小二乘，斜率的相反数就是接近速度
static float calcSpeed() {
  // 拟合窗必须填满才算数，否则返回 0 会被误当成"速度为零"
  if (s_fitN < FIT_N) return 0.0f;

  int start = (s_fitIdx - s_fitN + FIT_N * 2) % FIT_N;   // 从最旧的一笔开始
  uint32_t t0 = s_fitT[start];

  double st = 0, sd = 0, stt = 0, std_ = 0;
  for (int k = 0; k < s_fitN; k++) {
    int i = (start + k) % FIT_N;
    double t = (double)(s_fitT[i] - t0) / 1000.0;        // 秒
    double d = s_fitD[i];
    st += t; sd += d; stt += t * t; std_ += t * d;
  }
  double n   = (double)s_fitN;
  double den = n * stt - st * st;
  if (fabs(den) < 1e-9) return 0.0f;

  double slope = (n * std_ - st * sd) / den;             // 距离变化率（米/秒）
  return (float)(-slope);                                // 距离变小 = 靠近
}

static void clearFilters() {
  s_medN = s_avgN = s_fitN = s_winN = 0;
  s_medIdx = s_avgIdx = s_fitIdx = s_winIdx = 0;
  s_dist = -1.0f;
  s_speed = 0.0f;
}

/* ---------------- 告警判断（带去抖） ---------------- */

static void updateAlarm() {
  uint8_t want = 0x00;
  if (s_valid) {
    if (s_speed > BERTH_SPEED_ALARM)                        want = 0x02;
    else if (s_dist < BERTH_NEAR_M && s_speed > BERTH_NEAR_SPEED) want = 0x03;
    else if (s_speed > BERTH_SPEED_WARN)                    want = 0x01;
  }

  if (want == s_alarm) { s_candCode = 0; s_candCnt = 0; return; }

  if (want == s_candCode) {
    if (s_candCnt < 255) s_candCnt++;
  } else {
    s_candCode = want;
    s_candCnt  = 1;
  }

  if (s_candCnt >= BERTH_DEBOUNCE_N) {
    s_alarm    = want;
    s_candCode = 0;
    s_candCnt  = 0;
  }
}

/* ---------------- 播报 ---------------- */

static void berthAnnounce() {
  unsigned long now = millis();

  // 告警优先，且优先级高于距离播报
  if (s_alarm != 0x00) {
    if (now - s_lastAnnounceMs < BERTH_ALARM_GAP_MS) return;
    s_lastAnnounceMs = now;
    voiceSpeakBerthAlarm(s_alarm);
    return;
  }

  if (s_docked || !s_valid) return;

  // 静默：观察窗内几乎没变化，说明船停稳了，不必反复念同一个数字
  if (winRange() < BERTH_QUIET_M) return;

  uint32_t gap = 2000;                       // 近处播得勤一点
  if (s_dist >= 2.0f) gap = 2500;            // 远处慢一点
  if (now - s_lastAnnounceMs < gap) return;

  s_lastAnnounceMs = now;
  voiceSpeakBerthDistance(s_dist, s_dist < BERTH_NEAR_M);
}

static void berthEnd(const char* why) {
  s_active   = false;
  s_docked   = false;
  s_alarm    = 0x00;
  s_candCode = 0;
  s_candCnt  = 0;
  s_beyondRun = false;
  clearFilters();
  Serial.printf("[靠泊] 监测结束（%s）\n", why);
}

/* ---------------- 对外接口 ---------------- */

void berthBegin() {
  clearFilters();
  s_active = false;
  s_docked = false;
  s_alarm  = 0x00;
  s_lastValidMs = millis();
  Serial.println("靠泊辅助已就绪：靠近到 3.5 米自动开始监测。");
}

void berthUpdate() {
  unsigned long now = millis();
  if (now - s_lastSampleMs < TOF_READ_MS) return;
  s_lastSampleMs = now;

  s_valid = tofIsValid();

  if (s_valid) {
    float raw = (float)tofDistanceMm() / 1000.0f;

    // 1) 3 点中值：去野值
    s_med[s_medIdx] = raw;
    s_medIdx = (s_medIdx + 1) % MED_N;
    if (s_medN < MED_N) s_medN++;
    float med = (s_medN >= MED_N) ? median3(s_med[0], s_med[1], s_med[2]) : raw;

    // 2) 8 点平均：压噪声
    s_avg[s_avgIdx] = med;
    s_avgIdx = (s_avgIdx + 1) % AVG_N;
    if (s_avgN < AVG_N) s_avgN++;
    float sum = 0.0f;
    for (int i = 0; i < s_avgN; i++) sum += s_avg[i];
    s_dist = sum / (float)s_avgN;
    s_lastDist = s_dist;

    // 3) 送进拟合窗，算接近速度
    s_fitD[s_fitIdx] = s_dist;
    s_fitT[s_fitIdx] = now;
    s_fitIdx = (s_fitIdx + 1) % FIT_N;
    if (s_fitN < FIT_N) s_fitN++;
    s_speed = calcSpeed();

    // 4) 送进观察窗
    s_win[s_winIdx] = s_dist;
    s_winIdx = (s_winIdx + 1) % WIN_N;
    if (s_winN < WIN_N) s_winN++;

    s_lastValidMs = now;

    if (s_active && s_dist > s_maxDist) s_maxDist = s_dist;
  }

  /* ---- 激活 / 退出 ---- */
  if (!s_active) {
    if (s_valid && s_dist > 0.0f && s_dist <= BERTH_ENTER_M) {
      s_active   = true;
      s_docked   = false;
      s_alarm    = 0x00;
      s_candCode = 0;
      s_candCnt  = 0;
      s_beyondRun = false;
      s_undockMs  = 0;
      s_enterMs   = now;
      s_maxDist   = s_dist;
      voiceSpeakBerthEnter(s_dist);
      s_lastAnnounceMs = millis();
      Serial.printf("[靠泊] 进入监测，距离 %.2f 米\n", s_dist);
    }
    return;
  }

  /* 靠妥判定。四道防护缺一不可：
       拟合窗填满、观察窗攒到一半、进入监测满 2 秒、确实接近过 30 厘米以上。
     没有这几条防护，一开机就对着近处目标会被直接判成“刚刚靠好”。 */
  if (!s_docked && s_valid &&
      s_fitN >= FIT_N &&
      s_winN >= WIN_N / 2 &&
      (now - s_enterMs) >= BERTH_MIN_WATCH_MS &&
      (s_maxDist - s_dist) >= BERTH_APPROACH_MIN_M &&
      s_dist < BERTH_DONE_M &&
      fabsf(s_speed) < BERTH_DONE_SPEED &&
      winRange() < BERTH_DONE_STEADY_M) {
    s_docked = true;
    s_alarm  = 0x00;
    voiceSpeakBerthDone();
    s_lastAnnounceMs = millis();
    Serial.printf("[靠泊] 判定靠妥，距离 %.2f 米\n", s_dist);
  }

  // 激光量程下限是 4 厘米：刚才还贴着，读数突然没了，说明已经靠上
  if (!s_docked && !s_valid && s_lastDist > 0.0f && s_lastDist < 0.20f &&
      (s_maxDist - s_lastDist) >= BERTH_APPROACH_MIN_M &&
      (now - s_enterMs) >= BERTH_MIN_WATCH_MS &&
      (now - s_lastValidMs) > 300) {
    s_docked = true;
    voiceSpeakBerthDone();
    s_lastAnnounceMs = now;
    Serial.println("[靠泊] 读数进入盲区，判定已接触");
  }

  /* 靠妥之后又离开岸壁：解除“已靠妥”，回到监测状态 */
  if (s_docked && s_valid && s_dist > BERTH_UNDOCK_M) {
    if (s_undockMs == 0) s_undockMs = now;
    else if (now - s_undockMs >= BERTH_UNDOCK_MS) {
      s_docked   = false;
      s_undockMs = 0;
      s_maxDist  = s_dist;
      Serial.println("[靠泊] 已离开岸壁，恢复监测");
    }
  } else if (s_dist <= BERTH_UNDOCK_M) {
    s_undockMs = 0;
  }

  /* ---- 退出 ---- */
  if (s_valid && s_dist > BERTH_EXIT_M) {
    if (!s_beyondRun) { s_beyondRun = true; s_beyondMs = now; }
    else if (now - s_beyondMs >= BERTH_EXIT_MS) { berthEnd("已离开靠泊区"); return; }
  } else {
    s_beyondRun = false;
  }

  if (!s_valid && (now - s_lastValidMs) > 2000) { berthEnd("测距丢失"); return; }

  updateAlarm();
  berthAnnounce();
}

/* ---------------- 查询接口 ---------------- */

bool  berthActive()   { return s_active; }
bool  berthDocked()   { return s_docked; }
bool  berthValid()    { return s_valid; }
bool  berthShowOnScreen() {
  if (!s_active) return false;
  if (s_alarm != 0x00) return true;             // 有告警一定要显示
  if (s_docked)        return true;             // 已靠妥要显示
  return fabsf(s_speed) >= BERTH_SCREEN_SPEED;  // 正在移动才占用屏幕
}
float berthDistanceM(){ return s_dist; }
float berthSpeedMps() { return s_speed; }
uint8_t berthAlarmCode() { return s_alarm; }

String berthAlarmText() {
  switch (s_alarm) {
    case 0x01: return "接近速度偏大（提醒级）";
    case 0x02: return "接近速度过大（严重级）";
    case 0x03: return "距岸过近（严重级）";
    default:   return "无";
  }
}

String berthDistanceText() {
  char b[24];
  if (!s_valid || s_dist < 0.0f) return "无效";
  if (s_dist >= 1.0f) snprintf(b, sizeof(b), "%.2f m", s_dist);
  else                snprintf(b, sizeof(b), "%.0f cm", s_dist * 100.0f);
  return String(b);
}

void berthPrintReport() {
  if (!s_active) {
    if (s_valid && s_dist > 0.0f)
      Serial.printf("靠泊     : 待机（距岸 %s）\n", berthDistanceText().c_str());
    else
      Serial.println("靠泊     : 待机");
    return;
  }
  Serial.printf("靠泊     : %s  接近速度 %.3f m/s  告警 %s%s\n",
                berthDistanceText().c_str(), s_speed, berthAlarmText().c_str(),
                s_docked ? "  [已靠妥]" : "");
}

#else   /* BERTH_ENABLE == 0：全部空实现，不占代码 */

void berthBegin() {}
void berthUpdate() {}
bool berthActive() { return false; }
bool berthDocked() { return false; }
bool berthValid()  { return false; }
bool berthShowOnScreen() { return false; }
float berthDistanceM() { return -1.0f; }
float berthSpeedMps()  { return 0.0f; }
uint8_t berthAlarmCode() { return 0x00; }
String berthAlarmText() { return "无"; }
String berthDistanceText() { return "无效"; }
void berthPrintReport() {}

#endif
