/* =====================================================================
   shore.cpp   岸基节点跟踪实现
   ---------------------------------------------------------------------
   距离方位用的是和信标模块同一套球面公式（大圆距离 + 初始方位角）。
   岸基不告警、不播报，所以这里比 beacon.cpp 简单得多。
   ===================================================================== */

#include "shore.h"
#include "gnss.h"
#include "ownpos.h"     // 本船位置来源（北斗 / 手动坐标）
#include <math.h>

static const char* const GEO_DIR_UTF8[8] = { "正北", "东北", "正东", "东南",
                                             "正南", "西南", "正西", "西北" };

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

static int geoDirSector(double bearingDeg) {
  int s = (int)((bearingDeg + 22.5) / 45.0) % 8;
  if (s < 0) s += 8;
  return s;
}

/* ---------------- 状态 ---------------- */

static bool          s_linkUp  = false;
static unsigned long s_lastMs  = 0;
static bool          s_has     = false;
static bool          s_valid   = false;
static int           s_id      = 0;
static uint32_t      s_seq     = 0;
static double        s_lat = 0.0, s_lon = 0.0;
static int           s_rssi    = 0;
static float         s_snr     = 0.0f;

static bool          s_haveDir = false;
static float         s_distM   = 0.0f;
static float         s_bearing = 0.0f;
static int           s_sector  = 0;

// 用本船坐标算到岸基的方向与距离
static void refreshGeo() {
  s_haveDir = false;
  s_distM   = 0.0f;
  s_bearing = 0.0f;
  s_sector  = 0;

  /* 本船位置统一从 ownpos 取：默认北斗，室内演示时可能是手动坐标 */
  if (s_has && s_valid && ownPosValid()) {
    double mLat = ownPosLat();
    double mLon = ownPosLon();
    s_distM   = (float)geoDistanceM(mLat, mLon, s_lat, s_lon);
    s_bearing = (float)geoBearingDeg(mLat, mLon, s_lat, s_lon);
    s_sector  = geoDirSector(s_bearing);
    s_haveDir = true;
  }
}

/* ---------------- 对外接口 ---------------- */

void shoreBegin() {
  s_linkUp  = false;
  s_has     = false;
  s_valid   = false;
  s_id      = 0;
  s_haveDir = false;
}

void shoreOnPacket(const TargetPacket& pkt) {
  /* 本船自己回的 A 帧（srcId == DEV_ID）不是岸基发的，忽略。
     岸基的 ID 是 21/22…，和船端(1)、信标(11…)不重叠。 */
  if (pkt.srcId == DEV_ID) return;

  bool wasDown = !s_linkUp;
  s_lastMs = millis();
  s_linkUp = true;

  if (pkt.duplicate) {
    Serial.printf("[岸基 %d] #%lu 是重传包，已忽略\n", pkt.srcId, (unsigned long)pkt.seq);
    return;
  }

  s_has   = true;
  s_valid = pkt.valid;
  s_id    = pkt.srcId;
  s_seq   = pkt.seq;
  s_lat   = pkt.lat;
  s_lon   = pkt.lon;
  s_rssi  = pkt.rssi;
  s_snr   = pkt.snr;

  Serial.printf("[岸基 %d] 收到 %c #%lu  定位=%s  %.6f, %.6f  RSSI=%d dBm  SNR=%.1f dB\n",
                pkt.srcId, pkt.kind, (unsigned long)pkt.seq,
                pkt.valid ? "有效" : "无效", pkt.lat, pkt.lon, pkt.rssi, pkt.snr);

  refreshGeo();

  if (wasDown && s_has) {
    Serial.println("[岸基] 链路恢复");
  }
}

void shoreUpdate() {
  if (s_linkUp && (millis() - s_lastMs > SHORE_LOST_MS)) {
    s_linkUp = false;
    Serial.println("[岸基] 超过 15 秒没收到岸基帧");
  }
  refreshGeo();
}

void shorePrintReport() {
  Serial.println("--------------- 岸基节点 ---------------");
  Serial.printf("链路     : %s", s_linkUp ? "在线" : "离线");
  if (s_has) {
    Serial.printf("   编号 %d（%s）", s_id, s_id > 0 ? "带编号" : "老格式没带编号");
  }
  Serial.println();

  if (!s_has) {
    Serial.println("岸基位置 : 还没收到数据（岸基节点没上电或不在范围内）");
  } else {
    Serial.printf("岸基位置 : %s  #%lu  %.6f, %.6f\n",
                  s_valid ? "有效" : "未定位",
                  (unsigned long)s_seq, s_lat, s_lon);
    if (s_haveDir) {
      Serial.printf("距岸基   : %s方向  约 %.0f 米（绝对方位 %.0f 度）\n",
                    GEO_DIR_UTF8[s_sector], s_distM, s_bearing);
    } else {
      Serial.println("距岸基   : 等本船定位和岸基坐标都有效后给出");
    }
  }
  Serial.println("--------------------------------------");
}

bool        shoreLinkUp()  { return s_linkUp; }
bool        shoreHas()     { return s_has; }
bool        shoreValid()   { return s_valid; }
int         shoreId()      { return s_id; }
uint32_t    shoreSeq()     { return s_seq; }
double      shoreLat()     { return s_lat; }
double      shoreLon()     { return s_lon; }
int         shoreRssi()    { return s_rssi; }
float       shoreSnr()     { return s_snr; }
bool        shoreHaveDir() { return s_haveDir; }
float       shoreDistM()   { return s_distM; }
float       shoreBearing() { return s_bearing; }
const char* shoreDirText() { return s_haveDir ? GEO_DIR_UTF8[s_sector] : "--"; }
