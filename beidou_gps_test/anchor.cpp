/* =====================================================================
   anchor.cpp   走锚监测实现
   ---------------------------------------------------------------------
   处理链：
     北斗定位 → 位置进环 → 30 秒滑动平均 → 平滑位置 → 到基准的距离（位移）
     → 位移进环（60 秒）→ 分段最小二乘求增长率 → 判 0x21 / 0x22

   为什么平滑的是“位置”而不是“位移”：位移恒为非负，船在基准附近摆动时，
   位移的平均值会明显大于 0（正偏置），越平均越像“漂走了”。对经纬度做平均
   再算距离就没有这个问题。
   ===================================================================== */

#include "anchor.h"
#include "gnss.h"
#include "ownpos.h"     // 手动坐标期间挡住走锚监测
#include "tof.h"
#include "voice.h"
#include "beacon.h"     // 落水告警期间锚泊语音让位

#if ANCHOR_ENABLE

/* ---------------- 基准与历史 ---------------- */

static bool   s_hasRef  = false;
static double s_refLat  = 0.0, s_refLon = 0.0;
static float  s_refTofM = -1.0f;          // 设基准时的激光距离（无效则 -1）

static double s_posLat[ANCHOR_POS_N], s_posLon[ANCHOR_POS_N];
static int    s_posN = 0, s_posIdx = 0;

static float  s_hist[ANCHOR_HIST_N];
static int    s_histN = 0, s_histIdx = 0;

static float  s_drift = 0.0f, s_driftDir = 0.0f;
static float  s_slopeCur = 0.0f, s_slopePrev = 0.0f;

static uint8_t s_alarm = 0x00;
static bool    s_over = false;
static unsigned long s_overMs = 0;
static bool    s_laserOver = false;
static unsigned long s_laserOverMs = 0;

static unsigned long s_lastSampleMs = 0;
static unsigned long s_lastAnnounceMs = 0;

/* ---------------- 几何 ---------------- */

static double toRad(double d) { return d * M_PI / 180.0; }
static double toDeg(double r) { return r * 180.0 / M_PI; }

static double geoDist(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double p1 = toRad(lat1), p2 = toRad(lat2);
  double dp = toRad(lat2 - lat1);
  double dl = toRad(lon2 - lon1);
  double a = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
  if (a > 1.0) a = 1.0;
  return 2.0 * R * asin(sqrt(a));
}

static double geoBear(double lat1, double lon1, double lat2, double lon2) {
  double p1 = toRad(lat1), p2 = toRad(lat2);
  double dl = toRad(lon2 - lon1);
  double y = sin(dl) * cos(p2);
  double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
  double b = toDeg(atan2(y, x));
  if (b < 0) b += 360.0;
  return b;
}

// 方位角归到 8 个扇区（和语音词表 GB_DIR 的顺序一致）
static int dirSector(float bearingDeg) {
  int s = (int)((bearingDeg + 22.5f) / 45.0f) % 8;
  if (s < 0) s += 8;
  return s;
}

/* ---------------- 平滑与趋势 ---------------- */

static bool smoothedPos(double& lat, double& lon) {
  if (s_posN == 0) return false;
  double sla = 0.0, slo = 0.0;
  for (int i = 0; i < s_posN; i++) { sla += s_posLat[i]; slo += s_posLon[i]; }
  lat = sla / s_posN;
  lon = slo / s_posN;
  return true;
}

// 位移历史里做最小二乘，求增长率（米每秒）。
// fromNewest 表示从最新一笔往前跳过几笔，count 是用多少笔。
static float histSlope(int fromNewest, int count) {
  if (s_histN < fromNewest + count) return 0.0f;

  const double dt = ANCHOR_SAMPLE_MS / 1000.0;
  double st = 0, sd = 0, stt = 0, std_ = 0;
  for (int k = 0; k < count; k++) {
    int idx = (s_histIdx - 1 - fromNewest - k + ANCHOR_HIST_N * 2) % ANCHOR_HIST_N;
    double t = (count - 1 - k) * dt;         // 越新时间越大
    double d = s_hist[idx];
    st += t; sd += d; stt += t * t; std_ += t * d;
  }
  double n = count;
  double den = n * stt - st * st;
  if (fabs(den) < 1e-9) return 0.0f;
  return (float)((n * std_ - st * sd) / den);
}

/* ---------------- 判定与播报 ---------------- */

static void judge(unsigned long now) {
  /* 激光辅助：横向位移。停靠码头时比 GNSS 灵敏得多 */
  bool laserBig = false;
  if (s_refTofM > 0.0f && tofIsValid()) {
    float d = (float)tofDistanceMm() / 1000.0f;
    laserBig = (fabsf(d - s_refTofM) > ANCHOR_LASER_D_M);
  }

  /* 位移超限计时（带滞回，回到解除阈值以下就清零） */
  if (s_drift > ANCHOR_DRIFT_M) {
    if (!s_over) { s_over = true; s_overMs = now; }
  } else if (s_drift < ANCHOR_DRIFT_OFF_M) {
    s_over = false;
  }

  if (laserBig) {
    if (!s_laserOver) { s_laserOver = true; s_laserOverMs = now; }
  } else {
    s_laserOver = false;
  }

  const bool held    = s_over && (now - s_overMs >= ANCHOR_HOLD_MS);
  const bool laserOk = s_laserOver && (now - s_laserOverMs >= ANCHOR_LASER_MS);

  uint8_t want = 0x00;
  if (s_over && s_slopeCur > ANCHOR_SLOPE_DRAG && s_slopeCur >= s_slopePrev) {
    want = 0x22;      // 位移在持续增大，而且速率没有放缓
  } else if (held || laserOk) {
    want = 0x21;      // 超限持续够久，或者激光测到明显的横向位移
  }

  if (want != s_alarm) {
    /* 正在念同级（别的告警）或更高级（落水）就等一等；只是距离播报就直接顶掉。 */
    if (voiceBusyPrio() >= VOICE_PRIO_ALARM) return;
    s_alarm = want;
    s_lastAnnounceMs = now;
    /* 落水告警没确认期间只更新状态、不说话（屏幕和蜂鸣器照常） */
    if (!beaconAlarmActive()) {
      if      (want == 0x22) voiceSpeakAnchorDragging();
      else if (want == 0x21) voiceSpeakAnchorSuspect(s_drift, dirSector(s_driftDir));
      else if (want == 0x00) voiceSpeakAnchorOk();
    }
    Serial.printf("[锚泊] 告警状态变为 %02X（位移 %.2f 米，速率 %.3f 米每秒）\n",
                  want, s_drift, s_slopeCur);
    return;
  }

  // 告警持续时定期重播，免得漏听
  if (s_alarm != 0x00 && (now - s_lastAnnounceMs >= ANCHOR_REPEAT_MS)) {
    if (voiceBusyPrio() >= VOICE_PRIO_ALARM) return;
    s_lastAnnounceMs = now;
    if (beaconAlarmActive()) return;      // 落水没确认就先不抢语音
    if      (s_alarm == 0x22) voiceSpeakAnchorDragging();
    else if (s_alarm == 0x21) voiceSpeakAnchorSuspect(s_drift, dirSector(s_driftDir));
  }
}

/* ---------------- 对外接口 ---------------- */

void anchorBegin() {
  s_hasRef = false;
  s_posN = s_posIdx = 0;
  s_histN = s_histIdx = 0;
  s_alarm = 0x00;
  s_over = s_laserOver = false;
  Serial.println("走锚监测已就绪：用网页上的“设基准”按钮开始监测。");
}

void anchorUpdate() {
  if (!s_hasRef) return;

  unsigned long now = millis();
  if (now - s_lastSampleMs < ANCHOR_SAMPLE_MS) return;
  s_lastSampleMs = now;

  const GpsStatus& g = gpsGet();
  if (g.valid) {
    s_posLat[s_posIdx] = g.lat;
    s_posLon[s_posIdx] = g.lon;
    s_posIdx = (s_posIdx + 1) % ANCHOR_POS_N;
    if (s_posN < ANCHOR_POS_N) s_posN++;

    double lat, lon;
    if (smoothedPos(lat, lon)) {
      s_drift    = (float)geoDist(s_refLat, s_refLon, lat, lon);
      s_driftDir = (float)geoBear(s_refLat, s_refLon, lat, lon);

      s_hist[s_histIdx] = s_drift;
      s_histIdx = (s_histIdx + 1) % ANCHOR_HIST_N;
      if (s_histN < ANCHOR_HIST_N) s_histN++;

      s_slopeCur  = histSlope(0, 15);    // 最近 30 秒
      s_slopePrev = histSlope(15, 15);   // 再往前 30 秒
    }
  }

  judge(now);
}

void anchorSetReference() {
  const GpsStatus& g = gpsGet();

  /* 手动坐标是钉死的，位移永远是 0，走锚监测没意义 —— 直接挡住并说明 */
  if (ownPosManual()) {
    Serial.println("[锚泊] 现在用的是手动坐标（位置不会变），走锚监测需要真实北斗定位。");
    Serial.println("[锚泊] 想测走锚：串口敲 pos auto 切回北斗，等定位成功再设基准。");
    return;
  }

  if (!g.valid) {
    Serial.println("[锚泊] 现在没有有效定位，无法设基准");
    return;
  }

  s_refLat = g.lat;
  s_refLon = g.lon;
  s_refTofM = tofIsValid() ? (float)tofDistanceMm() / 1000.0f : -1.0f;

  s_hasRef  = true;
  s_posN = s_posIdx = 0;
  s_histN = s_histIdx = 0;
  s_drift = 0.0f; s_driftDir = 0.0f;
  s_slopeCur = s_slopePrev = 0.0f;
  s_alarm = 0x00;
  s_over = s_laserOver = false;
  s_lastSampleMs = millis();

  Serial.printf("[锚泊] 基准已设定：%.6f, %.6f", s_refLat, s_refLon);
  if (s_refTofM > 0.0f) Serial.printf("，当时离岸 %.2f 米", s_refTofM);
  Serial.println();

  voiceSpeakAnchorOn();
}

void anchorClearReference() {
  s_hasRef = false;
  s_alarm  = 0x00;
  s_over = s_laserOver = false;
  Serial.println("[锚泊] 已清除基准，停止监测");
}

bool   anchorActive()      { return s_hasRef; }
double anchorRefLat()      { return s_refLat; }
double anchorRefLon()      { return s_refLon; }
float  anchorDriftM()      { return s_drift; }
float  anchorDriftDir()    { return s_driftDir; }
float  anchorDriftSpeed()  { return s_slopeCur; }
uint8_t anchorAlarmCode()  { return s_alarm; }

String anchorAlarmText() {
  switch (s_alarm) {
    case 0x21: return "疑似走锚（提醒级）";
    case 0x22: return "走锚（严重级）";
    default:   return "无";
  }
}

String anchorStateText() {
  if (!s_hasRef) return "未设基准";
  switch (s_alarm) {
    case 0x21: return "疑似走锚";
    case 0x22: return "走锚";
    default:   return "监测中";
  }
}

String anchorDriftHistory() {
  String s;
  for (int k = 0; k < s_histN; k++) {
    int idx = (s_histIdx - s_histN + k + ANCHOR_HIST_N * 2) % ANCHOR_HIST_N;
    if (k) s += ",";
    s += String(s_hist[idx], 2);
  }
  return s;
}

void anchorPrintReport() {
  if (!s_hasRef) {
    Serial.println("锚泊     : 未设基准（网页上点“设基准”开始监测）");
    return;
  }
  Serial.printf("锚泊     : 位移 %.2f m   漂移 %03.0f 度   速率 %.3f m/s   告警 %s\n",
                s_drift, s_driftDir, s_slopeCur, anchorAlarmText().c_str());
}

#else   /* ANCHOR_ENABLE == 0 */

void anchorBegin() {}
void anchorUpdate() {}
void anchorSetReference() {}
void anchorClearReference() {}
bool anchorActive() { return false; }
double anchorRefLat() { return 0; }
double anchorRefLon() { return 0; }
float anchorDriftM() { return 0; }
float anchorDriftDir() { return 0; }
float anchorDriftSpeed() { return 0; }
uint8_t anchorAlarmCode() { return 0; }
String anchorAlarmText() { return "无"; }
String anchorStateText() { return "未启用"; }
String anchorDriftHistory() { return ""; }
void anchorPrintReport() {}

#endif
