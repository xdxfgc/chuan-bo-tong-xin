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
#include "beacon.h"     // 落水告警期间靠泊语音让位，需要 beaconAlarmActive()

#if BERTH_ENABLE

#define MED_N 3          // 中值滤波点数
#define AVG_N 8          // 平均滤波点数
#define FIT_N 8          // 速度拟合窗口（8 × 60ms ≈ 0.5 秒，60ms = TOF_READ_MS）
#define WIN_N 64         // 平稳/静默观察窗（64 × 60ms ≈ 3.8 秒）

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
static bool     s_useSide   = false;   // 当前用的是不是右舷那一路
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

/* 一次性播报（进入监测、靠泊完成）错过就不会再有，
   所以不跳过、只延后：先记下来，等上一句念完再放出去。 */
static bool          s_pendingEnter  = false;
static bool          s_pendingDone   = false;

/* 上一次播报出去的距离：距离变化不到门槛就不开口，省一半的话 */
static bool          s_hasSayDist    = false;
static float         s_sayDist       = 0.0f;

/* 靠泊距离曲线：每秒记一个点，网页上画趋势（文档表20） */
static float         s_hist[BERTH_HIST_N];
static int           s_histN   = 0, s_histIdx = 0;
static unsigned long s_histMs  = 0;

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
    /* 带滞回的判断：当前生效的那一级用“解除阈值”，其它级用“触发阈值”。
       这样既不会在阈值附近反复跳，又不会耽误升级——
       比如当前是提醒级（0x01）、速度冲到 0.31，上面的 0x02 条件用触发阈值判断，
       立刻就能升上去。 */
    bool keep2 = (s_alarm == 0x02) ? (s_speed > BERTH_SPEED_ALARM_OFF)
                                   : (s_speed > BERTH_SPEED_ALARM);
    bool keep3 = (s_alarm == 0x03) ? (s_dist < BERTH_NEAR_M_OFF && s_speed > BERTH_NEAR_SPEED_OFF)
                                   : (s_dist < BERTH_NEAR_M     && s_speed > BERTH_NEAR_SPEED);
    bool keep1 = (s_alarm == 0x01) ? (s_speed > BERTH_SPEED_WARN_OFF)
                                   : (s_speed > BERTH_SPEED_WARN);

    if      (keep2) want = 0x02;
    else if (keep3) want = 0x03;
    else if (keep1) want = 0x01;
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

/* 把挂着的一次性播报放出去（进入监测 / 靠泊完成）。
   这两句错过就没了，所以策略是"等"而不是"跳过"：上一句没念完就再等下一轮。 */
static void berthFlushPending() {
  if (!s_pendingEnter && !s_pendingDone) return;

  /* 落水告警没确认期间，靠泊的话一律不说。
     这时候"靠妥/进入监测"已经不是重点了，直接作废，别等告警结束了才补一句。 */
  if (beaconAlarmActive()) {
    s_pendingEnter = false;
    s_pendingDone  = false;
    return;
  }
  if (voiceBusy()) return;                 // 上一句还在念，等着

  if (s_pendingEnter) {
    s_pendingEnter = false;
    voiceSpeakBerthEnter(s_dist, s_useSide);
  } else {
    s_pendingDone = false;
    voiceSpeakBerthDone(s_useSide);
  }
  s_lastAnnounceMs = millis();
}

static void berthAnnounce() {
  unsigned long now = millis();

  /* 落水告警没确认期间，靠泊语音全部让位：
     人在水里这件事比"距岸还有几米"重要得多。
     注意只是"语音让位" —— 靠泊判断、屏幕显示、蜂鸣器都照常。 */
  if (beaconAlarmActive()) return;

  /* 有挂着的一次性播报时，距离播报先让路 ——
     距离下一轮还有，那两句只有一次 */
  if (s_pendingEnter || s_pendingDone) return;

  // 告警优先，且优先级高于距离播报
  if (s_alarm != 0x00) {
    if (now - s_lastAnnounceMs < BERTH_ALARM_GAP_MS) return;
    /* 正在念的如果是同级（别的告警）或更高（落水），就等它念完；
       正在念的只是距离播报的话，直接顶掉 —— 报警优先。 */
    if (voiceBusyPrio() >= VOICE_PRIO_ALARM) return;
    s_lastAnnounceMs = now;
    voiceSpeakBerthAlarm(s_alarm, s_useSide);
    return;
  }

  if (s_docked || !s_valid) return;

  // 静默：拟合速度接近零，说明船没在动，不必反复念同一个数字。
  // 用速度而不是极差，这样浪造成的往复波动不会让播报停不下来。
  if (fabsf(s_speed) < BERTH_QUIET_SPEED) return;

  /* 变化门槛：距离没挪够就别开口。
     "话"报得比"念"还快，是上一句被掐断的根因，先从这里减一半。 */
  if (s_hasSayDist && fabsf(s_dist - s_sayDist) < BERTH_ANNOUNCE_STEP_M) return;

  uint32_t gap = 3000;                       // 近处 3 秒一句（原来 2 秒）
  if (s_dist >= 2.0f) gap = 3500;            // 远处再慢半秒
  if (now - s_lastAnnounceMs < gap) return;

  /* 上一句还没念完就跳过这一次。
     靠泊报的是"当前距离"，晚一两秒再念出来数字就过期了，排队没有意义，
     所以宁可少说一句，也不要半句话被下一句掐断。下一轮再报最新的距离。 */
  if (voiceBusy()) {
    static unsigned long lastSkipLog = 0;
    voiceNoteSkip();                     // 记一笔：网页上能看到"跳过了几次"
    if (now - lastSkipLog >= 3000) {
      lastSkipLog = now;
      Serial.println("[靠泊] 上一句话还没念完，本次距离播报跳过");
    }
    return;
  }

  s_lastAnnounceMs = now;
  s_sayDist        = s_dist;
  s_hasSayDist     = true;
  voiceSpeakBerthDistance(s_dist, s_dist < BERTH_NEAR_M, s_useSide);
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

/* ---------------- 选哪一路当靠泊依据 ----------------
   真实船舶靠泊是"侧靠"——船横过来贴码头，所以右舷那只能测到"船到码头"
   的横向距离，比船头那只更贴近真实场景。
   但船头那只也不能废：顶着靠的时候只有它看得见岸。
   所以默认自动选：右舷进到靠泊区就用右舷，否则退回船头。
   退回时用退出阈值做滞回，免得两个距离在 3.5 米附近来回跳、
   每跳一次就重开一次监测。                                            */
static bool pickSideSource() {
#if !TOF_SIDE_ENABLE
  return false;                                     // 没装第二只
#elif BERTH_SRC_MODE == BERTH_SRC_SIDE
  return (tofSideIsValid() && tofSideDistanceMm() > 0);
#elif BERTH_SRC_MODE == BERTH_SRC_BOW
  return false;                                     // 固定用船头
#else
  if (!tofSideIsReady()) return false;
  if (!tofSideIsValid() || tofSideDistanceMm() == 0) return false;
  float d = (float)tofSideDistanceMm() / 1000.0f;
  return s_useSide ? (d <= BERTH_EXIT_M) : (d <= BERTH_ENTER_M);
#endif
}

/* ---------------- 对外接口 ---------------- */

void berthBegin() {
  clearFilters();
  s_active = false;
  s_docked = false;
  s_alarm  = 0x00;
  s_pendingEnter = false;
  s_pendingDone  = false;
  s_hasSayDist   = false;
  s_lastValidMs = millis();
  Serial.println("靠泊辅助已就绪：右舷优先（侧靠），看不到岸时自动用船头，"
                 "靠近到 3.5 米开始监测。");
}

void berthUpdate() {
  unsigned long now = millis();
  if (now - s_lastSampleMs < TOF_READ_MS) return;
  s_lastSampleMs = now;

  /* 先把挂着的"进入监测 / 靠泊完成"放出去（等上一句念完），
     放在最前面，靠泊结束之后也照样能放。 */
  berthFlushPending();

  /* ---- 选数据源：右舷优先（侧靠），看不到岸就退回船头 ---- */
  bool useSide = pickSideSource();

  /* 中途换源说明"岸"从船头换到了船侧（或者反过来）。两路的基准不一样，
     硬接着算会把速度拟合顶出一个假尖峰，所以直接结束本次监测，
     当成一个新事件重新进入。 */
  if (s_active && useSide != s_useSide) berthEnd("测距源切换");
  s_useSide = useSide;

  s_valid = useSide ? tofSideIsValid() : tofIsValid();

  if (s_valid) {
    float raw = (float)(useSide ? tofSideDistanceMm() : tofDistanceMm()) / 1000.0f;

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

  /* ---- 距离曲线：每秒记一个点（不管在不在监测，网页上都有趋势看）---- */
  if (now - s_histMs >= BERTH_HIST_MS) {
    s_histMs = now;
    s_hist[s_histIdx] = (s_valid && s_dist >= 0.0f) ? s_dist : -1.0f;
    s_histIdx = (s_histIdx + 1) % BERTH_HIST_N;
    if (s_histN < BERTH_HIST_N) s_histN++;
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
      s_pendingEnter = true;        // 不插话：等上一句念完再报（下一轮 loop 放）
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
    s_pendingDone = true;           // 同上：等上一句念完
    s_lastAnnounceMs = millis();
    Serial.printf("[靠泊] 判定靠妥，距离 %.2f 米\n", s_dist);
  }

  // 激光量程下限是 4 厘米：刚才还贴着，读数突然没了，说明已经靠上
  if (!s_docked && !s_valid && s_lastDist > 0.0f && s_lastDist < 0.20f &&
      (s_maxDist - s_lastDist) >= BERTH_APPROACH_MIN_M &&
      (now - s_enterMs) >= BERTH_MIN_WATCH_MS &&
      (now - s_lastValidMs) > 300) {
    s_docked = true;
    s_pendingDone = true;
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
bool  berthUsingSide(){ return s_useSide; }
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

/* 距离曲线：把环形缓冲按"从旧到新"拼成逗号分隔的字符串。
   无效点输出 -1，网页画图时在那里断开。 */
String berthDistanceHistory() {
  String s;
  s.reserve(s_histN * 6 + 8);
  for (int k = 0; k < s_histN; k++) {
    int idx = (s_histIdx - s_histN + k + BERTH_HIST_N * 2) % BERTH_HIST_N;
    if (k) s += ",";
    if (s_hist[idx] < 0.0f) s += "-1";
    else                    s += String(s_hist[idx], 2);
  }
  return s;
}

void berthPrintReport() {
  const char* src = s_useSide ? "右舷" : "船头";
  if (!s_active) {
    if (s_valid && s_dist > 0.0f)
      Serial.printf("靠泊(%s) : 待机（距岸 %s）\n", src, berthDistanceText().c_str());
    else
      Serial.printf("靠泊(%s) : 待机\n", src);
    return;
  }
  Serial.printf("靠泊(%s) : %s  接近速度 %.3f m/s  告警 %s%s\n",
                src, berthDistanceText().c_str(), s_speed, berthAlarmText().c_str(),
                s_docked ? "  [已靠妥]" : "");
}

#else   /* BERTH_ENABLE == 0：全部空实现，不占代码 */

void berthBegin() {}
void berthUpdate() {}
bool berthActive() { return false; }
bool berthDocked() { return false; }
bool berthValid()  { return false; }
bool berthUsingSide() { return false; }
bool berthShowOnScreen() { return false; }
float berthDistanceM() { return -1.0f; }
float berthSpeedMps()  { return 0.0f; }
uint8_t berthAlarmCode() { return 0x00; }
String berthAlarmText() { return "无"; }
String berthDistanceText() { return "无效"; }
String berthDistanceHistory() { return String(); }
void berthPrintReport() {}

#endif
