/* =====================================================================
   logbook.cpp   数据记录实现
   ---------------------------------------------------------------------
   存的是紧凑结构体，导出时才格式化成 CSV，省内存。
   坐标用 int32（×1e6）存，距离/方位用整数，够用又不占地方。
   ===================================================================== */

#include "logbook.h"
#include "gnss.h"
#include "tof.h"
#include "track.h"

#if LOGBOOK_ENABLE

struct LogRec {
  uint32_t tMs;        // 上电以来的毫秒
  uint32_t todSec;     // UTC 当日秒数（0 = 当时没有定位）

  int32_t  lat6, lon6;   // 本节点（×1e6）
  int32_t  bLat6, bLon6; // 主信标（×1e6）
  int32_t  vLat6, vLon6; // 船端（×1e6）

  uint16_t tofMm;      // 岸侧测距（毫米，0xFFFF = 无效）
  uint16_t bDistM;     // 距信标（米）
  uint16_t vDistM;     // 距船端（米）
  uint16_t bBrg;       // 信标绝对方位（度）
  uint16_t vBrg;       // 船端绝对方位（度）
  uint16_t vSog10;     // 船端对地速度 ×10（节）
  uint16_t vCog;       // 船端对地航向（度）

  uint8_t  sats;       // 本节点卫星数
  uint8_t  hdop10;     // 本节点 HDOP ×10
  uint8_t  fix;        // 本节点定位质量
  uint8_t  bId;        // 主信标编号
  uint8_t  bCount;     // 在线信标只数
  uint8_t  bLink;      // 主信标链路
  uint8_t  bValid;     // 主信标定位是否有效
  uint8_t  vLink;      // 船端链路
  uint8_t  vValid;     // 船端定位是否有效
  uint8_t  vSats;      // 船端卫星数
};

static LogRec   s_buf[LOGBOOK_CAPACITY];
static uint16_t s_head  = 0;
static uint16_t s_count = 0;
static bool     s_on    = true;
static unsigned long s_lastMs = 0;

// 取第 k 条（0 = 最旧）在环形缓冲里的下标
static uint16_t idxOf(uint16_t k) {
  return (uint16_t)((s_head - s_count + k + LOGBOOK_CAPACITY * 2) % LOGBOOK_CAPACITY);
}

static void fmtTime(uint32_t todSec, char* out, size_t n) {
  if (todSec == 0) { snprintf(out, n, "--:--:--"); return; }
  uint32_t bj = (todSec + 8 * 3600UL) % 86400UL;      // 换算成北京时间
  snprintf(out, n, "%02lu:%02lu:%02lu",
           (unsigned long)(bj / 3600), (unsigned long)((bj / 60) % 60),
           (unsigned long)(bj % 60));
}

/* ---------------- 对外接口 ---------------- */

void logbookBegin() {
  s_head = s_count = 0;
  s_on = true;
  s_lastMs = millis();
  Serial.println("数据记录已启动：每秒一帧，网页「⑤ 数据记录」可下载 CSV。");
}

void logbookUpdate() {
  if (!s_on) return;
  if (millis() - s_lastMs < LOG_INTERVAL_MS) return;
  s_lastMs = millis();

  const GpsStatus& g = gpsGet();
  const TrackTarget& b = trackBeacon();      // 最近活跃的那只信标
  const TrackTarget& v = trackVessel();
  LogRec& r = s_buf[s_head];

  r.tMs    = millis();
  r.todSec = (g.year != 0) ? (uint32_t)(g.hour * 3600UL + g.minute * 60UL + g.second) : 0;

  r.lat6 = (int32_t)lround(g.lat * 1000000.0);
  r.lon6 = (int32_t)lround(g.lon * 1000000.0);

  r.bLat6 = b.has ? (int32_t)lround(b.lat * 1000000.0) : 0;
  r.bLon6 = b.has ? (int32_t)lround(b.lon * 1000000.0) : 0;
  r.vLat6 = v.has ? (int32_t)lround(v.lat * 1000000.0) : 0;
  r.vLon6 = v.has ? (int32_t)lround(v.lon * 1000000.0) : 0;

  r.tofMm   = tofIsValid() ? tofDistanceMm() : 0xFFFF;
  r.bDistM  = b.haveDir ? (uint16_t)constrain(b.distM, 0.0f, 65535.0f) : 0;
  r.vDistM  = v.haveDir ? (uint16_t)constrain(v.distM, 0.0f, 65535.0f) : 0;
  r.bBrg    = b.haveDir ? (uint16_t)constrain(b.bearing, 0.0f, 359.9f) : 0;
  r.vBrg    = v.haveDir ? (uint16_t)constrain(v.bearing, 0.0f, 359.9f) : 0;
  r.vSog10  = v.hasExtra ? (uint16_t)constrain(v.sogKnots * 10.0f, 0.0f, 65535.0f) : 0;
  r.vCog    = v.hasExtra ? (uint16_t)constrain(v.cogDeg, 0.0f, 359.9f) : 0;

  r.sats    = (uint8_t)constrain(g.satsUsed, 0, 255);
  r.hdop10  = (uint8_t)constrain(lround(g.hdop * 10.0f), 0, 255);
  r.fix     = (uint8_t)constrain(g.fixQuality, 0, 255);
  r.bId     = (uint8_t)constrain(b.id, 0, 255);
  r.bCount  = (uint8_t)constrain(trackBeaconCount(), 0, 255);
  r.bLink   = (b.linkUp ? 1 : 0);
  r.bValid  = (b.valid  ? 1 : 0);
  r.vLink   = (v.linkUp ? 1 : 0);
  r.vValid  = (v.valid  ? 1 : 0);
  r.vSats   = (uint8_t)constrain(v.sats, 0, 255);

  s_head = (uint16_t)((s_head + 1) % LOGBOOK_CAPACITY);
  if (s_count < LOGBOOK_CAPACITY) s_count++;

#if LOG_SERIAL_CSV
  Serial.print("[LOG] ");
  Serial.print(logbookLine((uint16_t)(s_count - 1)));
#endif
}

bool logbookOn() { return s_on; }

void logbookSetOn(bool on) {
  s_on = on;
  Serial.printf("[记录] 已%s\n", on ? "开始" : "暂停");
}

void logbookClear() {
  s_head = s_count = 0;
  Serial.println("[记录] 已清空");
}

uint16_t logbookCount()    { return s_count; }
uint16_t logbookCapacity() { return LOGBOOK_CAPACITY; }

uint32_t logbookSpanSec() {
  if (s_count < 2) return 0;
  uint32_t a = s_buf[idxOf(0)].tMs;
  uint32_t b = s_buf[idxOf((uint16_t)(s_count - 1))].tMs;
  return (b - a) / 1000UL;
}

String logbookHeader() {
  return String("t_ms,time,node_lat,node_lon,node_sats,node_hdop,node_fix,tof_mm,"
                "b_count,b_id,b_link,b_valid,b_lat,b_lon,b_dist_m,b_brg,"
                "v_link,v_valid,v_lat,v_lon,v_dist_m,v_brg,v_sog_kn,v_cog,v_sats");
}

String logbookLine(uint16_t k) {
  if (k >= s_count) return String();
  const LogRec& r = s_buf[idxOf(k)];

  char tm[12];
  fmtTime(r.todSec, tm, sizeof(tm));

  char tof[10];
  if (r.tofMm == 0xFFFF) strncpy(tof, "", sizeof(tof));
  else                   snprintf(tof, sizeof(tof), "%u", (unsigned)r.tofMm);

  char line[240];
  snprintf(line, sizeof(line),
           "%lu,%s,%.6f,%.6f,%u,%.1f,%u,%s,"
           "%u,%u,%u,%u,%.6f,%.6f,%u,%u,"
           "%u,%u,%.6f,%.6f,%u,%u,%.1f,%u,%u\n",
           (unsigned long)r.tMs, tm,
           r.lat6 / 1000000.0, r.lon6 / 1000000.0,
           (unsigned)r.sats, r.hdop10 / 10.0, (unsigned)r.fix, tof,
           (unsigned)r.bCount, (unsigned)r.bId, (unsigned)r.bLink, (unsigned)r.bValid,
           r.bLat6 / 1000000.0, r.bLon6 / 1000000.0,
           (unsigned)r.bDistM, (unsigned)r.bBrg,
           (unsigned)r.vLink, (unsigned)r.vValid,
           r.vLat6 / 1000000.0, r.vLon6 / 1000000.0,
           (unsigned)r.vDistM, (unsigned)r.vBrg,
           r.vSog10 / 10.0, (unsigned)r.vCog, (unsigned)r.vSats);
  return String(line);
}

String logbookCSV() {
  String s;
  s.reserve((size_t)s_count * 140 + 200);
  s += logbookHeader();
  s += "\n";
  for (uint16_t k = 0; k < s_count; k++) s += logbookLine(k);
  return s;
}

void logbookPrintStatus() {
  Serial.printf("记录     : %s   已有 %u 条（容量 %u），覆盖约 %lu 秒\n",
                s_on ? "记录中" : "已暂停",
                (unsigned)s_count, (unsigned)LOGBOOK_CAPACITY,
                (unsigned long)logbookSpanSec());
}

#else   /* LOGBOOK_ENABLE == 0 */

void logbookBegin() {}
void logbookUpdate() {}
bool logbookOn() { return false; }
void logbookSetOn(bool) {}
void logbookClear() {}
uint16_t logbookCount() { return 0; }
uint16_t logbookCapacity() { return 0; }
uint32_t logbookSpanSec() { return 0; }
String logbookHeader() { return String(); }
String logbookLine(uint16_t) { return String(); }
String logbookCSV() { return String(); }
void logbookPrintStatus() {}

#endif
