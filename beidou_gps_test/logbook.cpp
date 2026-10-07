/* =====================================================================
   logbook.cpp   数据记录实现
   ---------------------------------------------------------------------
   记录内容按文档“数据记录规范”的要求挑：时间戳、卫星数、精度因子、
   定位状态、系统读数（距离/姿态/航向/位移/告警）与链路状态。
   存的是紧凑结构体，导出时才格式化成 CSV，省内存。
   ===================================================================== */

#include "logbook.h"
#include "gnss.h"
#include "ownpos.h"     // 本船位置来源（北斗 / 手动坐标）
#include "tof.h"
#include "imu.h"
#include "mag.h"
#include "berth.h"
#include "anchor.h"
#include "beacon.h"

#if LOGBOOK_ENABLE

struct LogRec {
  uint32_t tMs;        // 上电以来的毫秒
  uint32_t todSec;     // UTC 当日秒数（0 = 当时没有定位）
  int32_t  lat6;       // 纬度 ×1e6
  int32_t  lon6;       // 经度 ×1e6
  uint16_t tofMm;      // 激光距离（毫米）
  uint16_t tof2Mm;     // 右舷激光距离（毫米，0xFFFF = 无效）
  int16_t  pitch10;    // 俯仰 ×10
  int16_t  roll10;     // 横滚 ×10
  uint16_t head10;     // 航向 ×10
  uint16_t driftCm;    // 锚泊位移（厘米）
  uint16_t beaconM;    // 信标距离（米）
  uint8_t  sats;
  uint8_t  hdop10;     // HDOP ×10
  uint8_t  fix;        // 定位质量
  uint8_t  berth;      // 靠泊告警码
  uint8_t  anchor;     // 锚泊告警码
  uint8_t  link;       // 信标链路是否在线
};

static LogRec   s_buf[LOGBOOK_CAPACITY];
static uint16_t s_head  = 0;      // 下一个写入位置
static uint16_t s_count = 0;      // 已有条数
static bool     s_on    = true;
static unsigned long s_lastMs = 0;

/* ---------------- 内部 ---------------- */

// 取第 k 条（0 = 最旧）在环形缓冲里的下标
static uint16_t idxOf(uint16_t k) {
  return (uint16_t)((s_head - s_count + k + LOGBOOK_CAPACITY * 2) % LOGBOOK_CAPACITY);
}

static void fmtTime(uint32_t todSec, char* out, size_t n) {
  if (todSec == 0) { snprintf(out, n, "--:--:--"); return; }
  uint32_t bj = (todSec + 8 * 3600UL) % 86400UL;      // 换算成北京时间
  snprintf(out, n, "%02lu:%02lu:%02lu",
           (unsigned long)(bj / 3600), (unsigned long)((bj / 60) % 60), (unsigned long)(bj % 60));
}

/* ---------------- 对外接口 ---------------- */

void logbookBegin() {
  s_head = s_count = 0;
  s_on = true;
  s_lastMs = millis();
  Serial.println("数据记录已启动：每秒一帧，网页上可下载与回放。");
}

void logbookUpdate() {
  if (!s_on) return;
  if (millis() - s_lastMs < LOG_INTERVAL_MS) return;
  s_lastMs = millis();

  const GpsStatus& g = gpsGet();
  LogRec& r = s_buf[s_head];

  r.tMs      = millis();
  r.todSec   = (g.year != 0) ? (uint32_t)(g.hour * 3600UL + g.minute * 60UL + g.second) : 0;
  /* 位置记"系统认为的本船位置"：室内演示时可能是手动坐标，
     这样 CSV 里记的和网页上看到的是一致的。
     卫星数、HDOP、时间这些仍然是北斗的真实值。 */
  r.lat6     = (int32_t)lround(ownPosLat() * 1000000.0);
  r.lon6     = (int32_t)lround(ownPosLon() * 1000000.0);
  r.tofMm    = tofIsValid() ? tofDistanceMm() : 0xFFFF;
  r.tof2Mm   = (tofSideIsValid() && tofSideDistanceMm() > 0) ? tofSideDistanceMm() : 0xFFFF;
  r.pitch10  = (int16_t)lround(constrain(imuGet().pitch, -300.0f, 300.0f) * 10.0f);
  r.roll10   = (int16_t)lround(constrain(imuGet().roll,  -300.0f, 300.0f) * 10.0f);
  r.head10   = (magPresent() && magCalibrated())
                 ? (uint16_t)(lround(magHeadingDeg() * 10.0f) % 3600)
                 : 0xFFFF;                        // 没有可用航向
  r.driftCm  = (uint16_t)constrain(anchorDriftM() * 100.0f, 0.0f, 65535.0f);
  r.beaconM  = (uint16_t)constrain(beaconDistM(), 0.0f, 65535.0f);
  r.sats     = (uint8_t)constrain(g.satsUsed, 0, 255);
  r.hdop10   = (uint8_t)constrain(lround(g.hdop * 10.0f), 0, 255);
  r.fix      = (uint8_t)constrain(g.fixQuality, 0, 255);
  r.berth    = berthAlarmCode();
  r.anchor   = anchorAlarmCode();
  r.link     = (beaconLinkUp() ? 1 : 0);

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
  return String("t_ms,time,lat,lon,sats,hdop,fix,tof_mm,pitch,roll,heading,drift_m,"
                "berth,anchor,link,beacon_m,tof2_mm");
}

String logbookLine(uint16_t k) {
  if (k >= s_count) return String();
  const LogRec& r = s_buf[idxOf(k)];

  char tm[12];
  fmtTime(r.todSec, tm, sizeof(tm));

  char tof[10], tof2[10], head[10];
  if (r.tofMm == 0xFFFF) strncpy(tof, "", sizeof(tof));
  else                   snprintf(tof, sizeof(tof), "%u", (unsigned)r.tofMm);
  if (r.tof2Mm == 0xFFFF) strncpy(tof2, "", sizeof(tof2));
  else                    snprintf(tof2, sizeof(tof2), "%u", (unsigned)r.tof2Mm);
  if (r.head10 == 0xFFFF) strncpy(head, "", sizeof(head));
  else                    snprintf(head, sizeof(head), "%.1f", r.head10 / 10.0);

  char line[176];
  snprintf(line, sizeof(line),
           "%lu,%s,%.6f,%.6f,%u,%.1f,%u,%s,%+.1f,%+.1f,%s,%.2f,%u,%u,%u,%u,%s\n",
           (unsigned long)r.tMs, tm,
           r.lat6 / 1000000.0, r.lon6 / 1000000.0,
           (unsigned)r.sats, r.hdop10 / 10.0, (unsigned)r.fix,
           tof,
           r.pitch10 / 10.0, r.roll10 / 10.0, head,
           r.driftCm / 100.0,
           (unsigned)r.berth, (unsigned)r.anchor, (unsigned)r.link,
           (unsigned)r.beaconM, tof2);
  return String(line);
}

String logbookCSV() {
  String s;
  s.reserve((size_t)s_count * 90 + 160);
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
