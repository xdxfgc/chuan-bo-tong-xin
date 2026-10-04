/* =====================================================================
   beacon.cpp   信标接收与搜索引导实现
   ===================================================================== */

#include "beacon.h"
#include "gnss.h"
#include "lora_link.h"
#include "voice.h"
#include "mag.h"

/* ---------------- 方位与距离解算（球面公式） ---------------- */

static const char* const GEO_DIR_UTF8[8] = { "正北", "东北", "正东", "东南", "正南", "西南", "正西", "西北" };

// 相对船头的八个方位：0 正前方 1 右前方 2 正右方 3 右后方 4 正后方 5 左后方 6 正左方 7 左前方
static const char* const GEO_REL_UTF8[8] = { "正前方", "右前方", "正右方", "右后方",
                                             "正后方", "左后方", "正左方", "左前方" };

static double toRad(double d) { return d * M_PI / 180.0; }
static double toDeg(double r) { return r * 180.0 / M_PI; }

// 两点间大圆距离（米）
static double geoDistanceM(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double p1 = toRad(lat1), p2 = toRad(lat2);
  double dp = toRad(lat2 - lat1);
  double dl = toRad(lon2 - lon1);
  double a = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
  if (a > 1.0) a = 1.0;
  return 2.0 * R * asin(sqrt(a));
}

// 从点 1 指向点 2 的方位角（度，0 = 正北，顺时针）
static double geoBearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double p1 = toRad(lat1), p2 = toRad(lat2);
  double dl = toRad(lon2 - lon1);
  double y = sin(dl) * cos(p2);
  double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
  double b = toDeg(atan2(y, x));
  if (b < 0) b += 360.0;
  return b;
}

// 方位角归到 8 个扇区：0 正北 1 东北 2 正东 3 东南 4 正南 5 西南 6 正西 7 西北
static int geoDirSector(double bearingDeg) {
  int s = (int)((bearingDeg + 22.5) / 45.0) % 8;
  if (s < 0) s += 8;
  return s;
}

static const char* geoDirText(int sector) {
  if (sector < 0 || sector > 7) return "--";
  return GEO_DIR_UTF8[sector];
}

// 把「相对船头的角度」归到八个扇区
static int relDirSector(float rel) {
  int s = (int)((rel + 22.5f) / 45.0f) % 8;
  if (s < 0) s += 8;
  return s;
}

/* ---------------- 状态 ---------------- */

static bool          s_linkUp        = false;
static unsigned long s_lastPacketMs  = 0;
static bool          s_hasTarget     = false;
static bool          s_targetValid   = false;
static uint32_t      s_targetSeq     = 0;
static uint8_t       s_targetId      = BEACON_ID_NONE;
static double        s_tLat = 0.0, s_tLon = 0.0;
static int           s_rssi = 0;
static float         s_snr  = 0.0f;

static bool          s_haveDir  = false;
static float         s_distM    = 0.0f;
static float         s_bearing  = 0.0f;
static int           s_sector   = 0;

static bool          s_useRel     = false;   // 是否能用相对船头方位
static float         s_relBearing = 0.0f;
static int           s_relSector  = 0;

static bool          s_announced      = false;   // 播报去重
static bool          s_reportedNoFix  = false;
static double        s_annLat = 0.0, s_annLon = 0.0;
static unsigned long s_lastAnnounceMs = 0;
static bool          s_lastHaveDir    = false;

static unsigned long s_lastLoraRetryMs = 0;

/* 编号文字：信标用 11/12/13……，本板是 1；老格式帧不带编号，显示 “--” */
static char s_idText[8] = "--";

static void updateIdText(uint8_t id) {
  if (id == BEACON_ID_NONE) snprintf(s_idText, sizeof(s_idText), "--");
  else                      snprintf(s_idText, sizeof(s_idText), "%u", (unsigned)id);
}

static bool          s_alarmActive = false;      // 落水告警活动（需人工确认）
static bool          s_acked       = false;      // 本次落水事件是否已确认
static bool          s_everLinked  = false;      // 上电以来是否收到过包

/* ---------------- 内部逻辑 ---------------- */

// 用本船坐标算到信标的方向与距离
static void refreshGeo() {
  s_haveDir = false;
  s_distM   = 0.0f;
  s_bearing = 0.0f;
  s_sector  = 0;
  s_useRel     = false;
  s_relBearing = 0.0f;
  s_relSector  = 0;

  const GpsStatus& g = gpsGet();
  if (s_hasTarget && s_targetValid && g.valid) {
    s_distM   = (float)geoDistanceM(g.lat, g.lon, s_tLat, s_tLon);
    s_bearing = (float)geoBearingDeg(g.lat, g.lon, s_tLat, s_tLon);
    s_sector  = geoDirSector(s_bearing);
    s_haveDir = true;

    // 相对船头：需要磁力计在位、而且标定过，否则退回绝对方位播报
    if (magPresent() && magCalibrated()) {
      float rel = s_bearing - magHeadingDeg();
      while (rel < 0.0f)     rel += 360.0f;
      while (rel >= 360.0f)  rel -= 360.0f;
      s_relBearing = rel;
      s_relSector  = relDirSector(rel);
      s_useRel     = true;
    }
  }
}

// 播报去重：同一条坐标不反复念，否则移动目标会把语音队列堆爆
static void maybeAnnounce() {
  if (s_announced && (millis() - s_lastAnnounceMs < ANNOUNCE_MIN_GAP_MS)) return;

  bool first     = !s_announced;
  bool dirChange = (s_haveDir != s_lastHaveDir);
  bool moved     = s_announced && s_haveDir &&
                   geoDistanceM(s_annLat, s_annLon, s_tLat, s_tLon) >= ANNOUNCE_MIN_MOVE_M;
  bool timeout   = s_announced && (millis() - s_lastAnnounceMs >= ANNOUNCE_MAX_MS);

  if (!(first || dirChange || moved || timeout)) return;

  voiceAnnounce(s_haveDir, s_tLat, s_tLon, s_distM,
                s_useRel ? s_relSector : s_sector, s_useRel);

  s_announced      = true;
  s_lastHaveDir    = s_haveDir;
  s_annLat         = s_tLat;
  s_annLon         = s_tLon;
  s_lastAnnounceMs = millis();
}

// 收到一包信标数据
static void onPacket(const TargetPacket& pkt) {
  bool wasDown = !s_linkUp;

  s_lastPacketMs = millis();
  s_linkUp       = true;
  s_everLinked   = true;

  // 先回 ACK，别被显示和播报拖慢
  const GpsStatus& g = gpsGet();
  loraSendAck(pkt.seq, g.valid, g.lat, g.lon);

  if (pkt.duplicate) {
    Serial.printf("[LoRa] 信标 %u #%lu 是重传包，已回 ACK（不重复播报）\n",
                  (unsigned)pkt.beaconId, (unsigned long)pkt.seq);
    return;
  }

  s_hasTarget   = true;
  s_targetValid = pkt.valid;
  s_targetSeq   = pkt.seq;
  s_targetId    = pkt.beaconId;
  updateIdText(pkt.beaconId);
  s_tLat        = pkt.lat;
  s_tLon        = pkt.lon;
  s_rssi        = pkt.rssi;
  s_snr         = pkt.snr;

  Serial.printf("[LoRa] 收到信标 %u #%lu  定位=%s  %.6f, %.6f  RSSI=%d dBm  SNR=%.1f dB\n",
                (unsigned)pkt.beaconId, (unsigned long)pkt.seq, pkt.valid ? "有效" : "无效",
                pkt.lat, pkt.lon, pkt.rssi, pkt.snr);

  if (wasDown && s_announced) voiceSpeakLinkBack();

  refreshGeo();

  if (s_targetValid) {
    // 落水告警：收到有效坐标就置位，直到人工确认；确认后同一个事件不再重复触发
    if (!s_acked) s_alarmActive = true;
    s_reportedNoFix = false;
    maybeAnnounce();
  } else if (!s_reportedNoFix) {
    s_reportedNoFix = true;
    voiceSpeakTargetNoPos();
  }
}

/* ---------------- 对外接口 ---------------- */

void beaconBegin() {
  s_linkUp = false;
  s_hasTarget = false;
  s_targetValid = false;
  s_targetId = BEACON_ID_NONE;
  updateIdText(BEACON_ID_NONE);
  s_announced = false;
  s_reportedNoFix = false;
  s_haveDir = false;
}

void beaconUpdate() {
  // LoRa：射频没起来就每 2 秒重试，起来了就收包
  if (!loraIsReady()) {
    if (millis() - s_lastLoraRetryMs >= 2000) {
      s_lastLoraRetryMs = millis();
      loraBegin();
    }
  } else {
    TargetPacket pkt;
    if (loraPoll(&pkt)) onPacket(pkt);
  }

  // 链路超时：这里必须重新读一次 millis()。刚收到的包会把 s_lastPacketMs
  // 设成「比本轮 now 还新」的时刻，用旧值相减会变成无符号下溢，刚收到包就被误判。
  if (s_linkUp && (millis() - s_lastPacketMs > LINK_LOST_MS)) {
    s_linkUp = false;
    s_acked  = false;          // 失联视为本次事件结束，之后恢复可重新报警
    Serial.println("[链路] 超过 15 秒没收到信标，播报失去联系");
    voiceSpeakLinkLost();
  }
}

void beaconPrintReport() {
  Serial.println("--------------- 信标链路 ---------------");
  Serial.printf("链路     : %s   RSSI %d dBm   SNR %.1f dB\n",
                s_linkUp ? "在线" : "离线", s_rssi, s_snr);
  if (s_hasTarget) {
    Serial.printf("信标编号 : %s（0 或 -- 表示老格式帧没带编号）\n", s_idText);
    Serial.printf("信标位置 : %s  #%lu  %.6f, %.6f\n",
                  s_targetValid ? "有效" : "未定位",
                  (unsigned long)s_targetSeq, s_tLat, s_tLon);
  } else {
    Serial.println("信标位置 : 还没收到数据");
  }
  if (s_haveDir) {
    if (s_useRel) {
      Serial.printf("搜索引导 : %s  约 %.0f 米（相对船头 %.0f 度 / 绝对 %.0f 度）\n",
                    GEO_REL_UTF8[s_relSector], s_distM, s_relBearing, s_bearing);
    } else {
      Serial.printf("搜索引导 : %s方向  约 %.0f 米（绝对方位 %.0f 度）\n",
                    geoDirText(s_sector), s_distM, s_bearing);
    }
  } else {
    Serial.println("搜索引导 : 等本船定位和信标坐标都有效后给出");
  }
  Serial.println("--------------------------------------");
}

bool        beaconLinkUp()      { return s_linkUp; }
bool        beaconHasTarget()   { return s_hasTarget; }
bool        beaconTargetValid() { return s_targetValid; }
uint32_t    beaconSeq()         { return s_targetSeq; }
uint8_t     beaconId()          { return s_targetId; }
const char* beaconIdText()      { return s_idText; }
double      beaconLat()         { return s_tLat; }
double      beaconLon()         { return s_tLon; }
int         beaconRssi()        { return s_rssi; }
float       beaconSnr()         { return s_snr; }
bool        beaconHaveDir()     { return s_haveDir; }
float       beaconDistM()       { return s_distM; }
float       beaconBearing()     { return s_bearing; }
const char* beaconDirText()     { return s_haveDir ? geoDirText(s_sector) : "--"; }

bool        beaconUseRel()      { return s_useRel; }
float       beaconRelBearing()  { return s_relBearing; }
const char* beaconRelDirText()  { return s_useRel ? GEO_REL_UTF8[s_relSector] : "--"; }
float       beaconArrowBearing(){ return s_useRel ? s_relBearing : s_bearing; }

bool beaconAlarmActive() { return s_alarmActive; }
bool beaconEverLinked()  { return s_everLinked; }

void beaconAcknowledge() {
  s_alarmActive = false;
  s_acked       = true;
}
